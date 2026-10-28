"""RoboMaster SDK bridge. Starts disarmed; explicit arm service after preflight."""
import json
import math
import os
import socket
import threading
import time
from pathlib import Path
import yaml
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from robomaster import robot, config
from .vision import LatestVideo, TagDetector


class Driver(Node):
    def __init__(self):
        super().__init__('robomaster_driver')
        self.declare_parameter('config_file', '')
        path = self.get_parameter('config_file').value
        self.cfg = yaml.safe_load(Path(path).read_text())
        self.armed = False
        self.latched_stop = False
        self.stop_state = 'LATCHED_STOP'
        self.lock = threading.Lock()
        self.command = dict(stamp=-1e9, vx=0, wz=0, state='DISARMED')
        self.latest = {}
        self.last_command_ack = False
        self.last_command_ack_stamp = 0.0
        self.last_ack_warning = 0.0
        self.tag_data = None
        self.last_frame = -1e9
        self.video = LatestVideo()
        cv2.setNumThreads(2)
        self.detector = TagDetector(self.cfg)
        self.vision_stop = threading.Event()
        self.vision_data = {}
        self.pub_tag = self.create_publisher(String, '/rm/tag', 1)
        self.pub_vision = self.create_publisher(String, '/rm/vision', 1)
        self.pub_sensors = self.create_publisher(String, '/rm/sensors', 1)
        self.pub_applied = self.create_publisher(String, '/rm/applied', 10)
        self.create_subscription(String, '/rm/command', self.receive_command, 1)
        self.create_service(SetBool, '/rm/arm', self.arm)
        self.create_service(Trigger, '/rm/stop', self.stop_service)
        self.ep = robot.Robot()
        self.connected = False
        self.sdk_subscriptions = []
        try:
            if self.cfg['local_ip']:
                config.LOCAL_IP_STR = self.cfg['local_ip']
            elif self.cfg['conn_type'] == 'ap':
                # Reuse the route-based interface selection proven in Assessment 1.
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    sock.connect(('192.168.2.1', 20020))
                    config.LOCAL_IP_STR = sock.getsockname()[0]
            if self.cfg['robot_ip']:
                config.ROBOT_IP_STR = self.cfg['robot_ip']
            if not self.ep.initialize(conn_type=self.cfg['conn_type']):
                raise RuntimeError('SDK initialization failed')
            self.connected = True
            self.ep.chassis.drive_speed(x=0, y=0, z=0)
            # Assessment's arm pose; verify mount and ToF clearance physically.
            action = self.ep.robotic_arm.moveto(
                x=self.cfg.get('arm_x_mm', 105), y=self.cfg.get('arm_y_mm', 30))
            if not action.wait_for_completed(timeout=10):
                raise RuntimeError('Arm initialization did not complete')
            for module, sub, unsub, name in [
                    (self.ep.sensor, 'sub_distance', 'unsub_distance', 'tof'),
                    (self.ep.chassis, 'sub_esc', 'unsub_esc', 'esc'),
                    (self.ep.chassis, 'sub_velocity', 'unsub_velocity', 'velocity')]:
                if not getattr(module, sub)(freq=20, callback=lambda data, n=name: self.receive(n, data)):
                    raise RuntimeError('Subscription failed: '+name)
                self.sdk_subscriptions.append((module, unsub))
            # Adapter retains SDK stream-control API, replaces decoder with OpenCV.
            self.ep.camera._liveview = self.video
            if not self.ep.camera.start_video_stream(display=False, resolution='720p'):
                raise RuntimeError('Video stream failed')
        except Exception:
            self.close()
            raise
        self.vision_thread = threading.Thread(target=self.process_vision, daemon=True)
        self.vision_thread.start()
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Connected, arm at (105,30), CHASSIS DISARMED.')

    def receive(self, name, data):
        with self.lock:
            self.latest[name] = (time.monotonic(), data)

    def receive_command(self, msg):
        self.command = json.loads(msg.data)

    def process_vision(self):
        while not self.vision_stop.is_set():
            stamp, frame = self.video.latest()
            if frame is None or stamp <= self.last_frame:
                self.vision_stop.wait(0.01)
                continue
            self.last_frame = stamp
            started = time.monotonic()
            try:
                detection = self.detector.detect(frame)
                diagnostic = dict(stamp=stamp, ids=self.detector.last_ids,
                                  processing_ms=1000*(time.monotonic()-started),
                                  calibration_verified=self.cfg['calibration_verified'],
                                  candidate_pose=detection)
                if detection is not None and self.cfg['calibration_verified']:
                    detection['stamp'] = stamp
                    self.tag_data = detection
                    self.pub_tag.publish(String(data=json.dumps(detection)))
                self.vision_data = diagnostic
                self.pub_vision.publish(String(data=json.dumps(diagnostic)))
                if self.detector.last_ids:
                    import numpy as np
                    cv2.aruco.drawDetectedMarkers(frame, self.detector.last_corners,
                                                 np.array(self.detector.last_ids).reshape(-1, 1))
                # Atomic replacement prevents the dashboard reading half a JPEG.
                cv2.imwrite('/tmp/rm_latest.tmp.jpg', frame)
                os.replace('/tmp/rm_latest.tmp.jpg', '/tmp/rm_latest.jpg')
            except Exception as exc:
                self.vision_data = dict(stamp=stamp, ids=[], error=str(exc))
                self.get_logger().error('Vision: '+str(exc))
            self.vision_stop.wait(0.01)

    def health(self, now):
        with self.lock:
            latest = self.latest.copy()
        # EP reports four zero slots when no ToF modules are registered. Require
        # fresh wheel telemetry and camera frames; ToF stays an additional guard
        # when it is available, with camera range as the primary stop guard.
        fresh = ('esc' in latest and now-latest['esc'][0] < 0.3
                 and now-self.video.stamp < 0.45
                 and self.last_command_ack
                 and now-self.last_command_ack_stamp < 0.5)
        raw_tof = latest.get('tof', (None, [0]*4))[1]
        mm = raw_tof[self.cfg['front_tof_index']]
        tof_fresh = 'tof' in latest and 0 <= now-latest['tof'][0] < 0.30
        tof = mm/1000.0 if tof_fresh and 0 < mm < 65535 else None
        return fresh, tof, latest

    def arm(self, request, response):
        if not request.data:
            self.armed = False
            self.ep.chassis.drive_speed(x=0, y=0, z=0)
            response.success, response.message = True, 'Disarmed'
            return response
        healthy, _, _ = self.health(time.monotonic())
        seen_recently = (self.tag_data is not None and
                         0 <= time.monotonic()-self.tag_data['stamp'] < 10)
        if (self.latched_stop or not healthy or not self.cfg['calibration_verified']
                or not seen_recently):
            response.success = False
            response.message = 'Requires verified calibration, live camera/ESC and Tag 3 seen within 10 seconds; stop must not be latched'
        else:
            self.armed = True
            response.success, response.message = True, 'Armed'
        return response

    def stop_service(self, request, response):
        self.armed, self.latched_stop = False, True
        self.ep.chassis.drive_speed(x=0, y=0, z=0)
        response.success, response.message = True, 'Stop latched; restart driver for another run'
        return response

    def tick(self):
        now = time.monotonic()
        healthy, tof, latest = self.health(now)
        self.pub_sensors.publish(String(data=json.dumps(dict(
            stamp=now, healthy=healthy, enabled=self.armed, tof_m=tof,
            calibration_verified=self.cfg['calibration_verified'],
            latched_stop=self.latched_stop,
            command_acknowledged=self.last_command_ack))))
        c = self.command.copy()
        tag_distance = None
        if self.tag_data is not None and 0 <= now-self.tag_data['stamp'] <= 0.45:
            tag_distance = self.tag_data['distance']
        if (tof is not None and tof <= 0.20) or (tag_distance is not None and tag_distance <= 0.20):
            self.latched_stop = True
            self.stop_state = 'ARRIVED'
            c.update(vx=0, wz=0, state='ARRIVED')
        elif self.latched_stop:
            c.update(vx=0, wz=0, state=self.stop_state)
        elif not self.armed:
            c.update(vx=0, wz=0, state='DISARMED')
        elif not healthy or not 0 <= now-c['stamp'] <= 0.25:
            c.update(vx=0, wz=0, state='WATCHDOG')
        elif c['vx'] > 0 and tag_distance is None:
            c.update(vx=0, wz=0, state='TAG_HOLD')
        if not all(math.isfinite(c[k]) for k in ('vx', 'wz')):
            c.update(vx=0, wz=0, state='INVALID_COMMAND')
        c['vx'] = min(0.16, max(0, c['vx']))
        c['wz'] = min(0.30, max(-0.30, c['wz']))
        # SDK z is clockwise degrees/s; ROS/body yaw is CCW radians/s.
        try:
            acknowledged = bool(self.ep.chassis.drive_speed(
                x=c['vx'], y=0, z=-math.degrees(c['wz']), timeout=0.25))
        except Exception as exc:
            acknowledged = False
            self.get_logger().warning(f'Chassis command failed: {exc}')
        self.last_command_ack = acknowledged
        if acknowledged:
            self.last_command_ack_stamp = time.monotonic()
        else:
            moving_request = c['vx'] != 0 or c['wz'] != 0
            c.update(vx=0, wz=0)
            if moving_request or self.armed:
                # A lost acknowledgement makes the applied velocity unknown.
                # Disarm, latch, and try an explicit zero command immediately.
                self.armed = False
                self.latched_stop = True
                self.stop_state = 'SDK_ACK_LOST'
                try:
                    self.ep.chassis.drive_speed(x=0, y=0, z=0, timeout=0.25)
                except Exception as exc:
                    self.get_logger().error(f'Zero-speed fallback failed: {exc}')
            c['state'] = 'SDK_ACK_LOST' if moving_request else 'DISARMED_NO_ACK'
            if time.monotonic()-self.last_ack_warning > 2:
                self.get_logger().warning(
                    'Chassis command acknowledgement missing; chassis remains disarmed and mission is inhibited.')
                self.last_ack_warning = time.monotonic()
        c.update(stamp=time.monotonic(), source='HARDWARE', tag_distance_m=tag_distance,
                 tof_m=tof, raw_sensors=latest,
                 command_acknowledged=acknowledged)
        velocity = latest.get('velocity')
        if velocity is not None and 0 <= now-velocity[0] < 0.30:
            c['vx_measured_mps'] = velocity[1][3]
        c['tag_bearing_rad'] = self.tag_data['bearing'] if tag_distance is not None else None
        self.pub_applied.publish(String(data=json.dumps(c)))

    def close(self):
        self.vision_stop.set()
        if self.connected:
            try:
                self.ep.chassis.drive_speed(x=0, y=0, z=0)
            except Exception:
                pass
        if hasattr(self, 'vision_thread'):
            self.vision_thread.join(timeout=2)
        self.video.stop()
        for module, unsub in self.sdk_subscriptions:
            try:
                getattr(module, unsub)()
            except Exception:
                pass
        try:
            self.ep.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Driver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
