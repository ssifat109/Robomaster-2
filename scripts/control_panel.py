"""Local recording/control panel for the existing ROS 2 hardware mission."""
import argparse
import json
import os
import secrets
import signal
import socket
import subprocess
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from zoneinfo import ZoneInfo

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from rm_flatness_nav.core import wheel_rates, motor_voltages

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'rm_flatness_nav/config/hardware.yaml'
PAGE = ROOT / 'scripts/control_panel.html'


class Panel(Node):
    def __init__(self, folder):
        super().__init__('assessment_control_panel')
        self.folder = folder
        self.session_folder = folder
        self.attempt = 0
        self.launch_log = None
        self.sensors = {}
        self.vision = {}
        self.applied = {}
        self.sensor_received = 0.0
        self.applied_received = 0.0
        self.last_browser = time.monotonic()
        self.deadline = None
        self.writer = None
        self.record_started = None
        self.recorded_frames = 0
        self.previous_rates = np.zeros(4)
        self.previous_stamp = None
        self.volts = [0.0]*4
        self.message = 'Connecting to robot. Arm initializes to (105, 30) mm.'
        self.launch = None
        self.mutex = threading.RLock()
        self.arm_client = self.create_client(SetBool, '/rm/arm')
        self.stop_client = self.create_client(Trigger, '/rm/stop')
        self.create_subscription(String, '/rm/sensors', self.on_sensors, 1)
        self.create_subscription(String, '/rm/vision', self.on_vision, 1)
        self.create_subscription(String, '/rm/applied', self.on_applied, 10)
        self.create_timer(0.1, self.tick)

    def on_sensors(self, msg):
        self.sensors = json.loads(msg.data)
        self.sensor_received = time.monotonic()

    def on_vision(self, msg):
        self.vision = json.loads(msg.data)

    def on_applied(self, msg):
        data = json.loads(msg.data)
        stamp = data['stamp']
        rates = wheel_rates(data['vx'], data['wz'])
        if self.previous_stamp is not None and stamp > self.previous_stamp:
            self.volts = motor_voltages(rates, self.previous_rates,
                                       stamp-self.previous_stamp).tolist()
        self.previous_stamp, self.previous_rates = stamp, rates
        self.applied, self.applied_received = data, time.monotonic()

    def status(self):
        now = time.monotonic()
        fresh = now-self.sensor_received < 0.5
        verified = self.sensors.get('calibration_verified', False)
        pose = self.vision.get('candidate_pose')
        seen = 3 in self.vision.get('ids', []) and now-self.vision.get('stamp', 0) < 10
        reasons = []
        if not fresh:
            reasons.append('Waiting for live robot telemetry')
        if not verified:
            reasons.append('Distance calibration is not verified')
        if not self.sensors.get('healthy', False):
            reasons.append('Camera, wheel telemetry, or chassis acknowledgement is not fresh')
        if not self.sensors.get('command_acknowledged', False):
            reasons.append('Chassis command not acknowledged; motion is inhibited')
        if not seen:
            reasons.append('Tag 3 has not been decoded recently')
        if self.sensors.get('latched_stop', False):
            reasons.append('Stop is latched; close and reopen start.bat for another run')
        if self.launch is not None and self.launch.poll() is not None:
            reasons.append('ROS hardware process exited; see launch.log below')
        return dict(ready=not reasons, reasons=reasons,
                    state=self.applied.get('state', 'CONNECTING') if fresh else 'OFFLINE',
                    enabled=self.sensors.get('enabled', False),
                    command_acknowledged=self.sensors.get('command_acknowledged', False),
                    ids=self.vision.get('ids', []),
                    vision_age_s=round(now-self.vision.get('stamp', now), 2),
                    calibration_verified=verified,
                    candidate_pose=pose, tof_m=self.sensors.get('tof_m'),
                    vx=self.applied.get('vx', 0), wz=self.applied.get('wz', 0),
                    model_voltages=self.volts, message=self.message,
                    recording=self.writer is not None,
                    recording_seconds=(round(now-self.record_started, 1)
                                       if self.writer is not None else None),
                    recorded_frames=self.recorded_frames, output=str(self.folder))

    def start_robot(self):
        with self.mutex:
            if self.launch is not None and self.launch.poll() is None:
                return dict(success=False, message='Robot process is already running')
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.connect(('192.168.2.1', 20020))
                address = sock.getsockname()[0]
            if not address.startswith('192.168.2.'):
                self.message = (f'WSL currently uses {address}. Connect Windows to RMEP-21b41f, '
                                'then click Retry connection. No robot command sent.')
                return dict(success=False, message=self.message)
            if self.writer is not None:
                self.writer.release()
                self.writer = None
            if self.launch_log is not None:
                self.launch_log.close()
            self.attempt += 1
            self.folder = self.session_folder / f'attempt_{self.attempt:02d}'
            self.folder.mkdir()
            self.sensors, self.vision, self.applied = {}, {}, {}
            self.sensor_received = self.applied_received = 0.0
            self.previous_stamp = None
            self.volts = [0.0]*4
            self.deadline = None
            Path('/tmp/rm_latest.jpg').unlink(missing_ok=True)
            self.launch_log = (self.folder/'launch.log').open('w')
            self.launch = subprocess.Popen(
                ['ros2', 'launch', 'rm_flatness_nav', 'mission.launch.py', 'sim:=false',
                 'config_file:='+str(CONFIG), 'output_dir:='+str(self.folder)],
                stdout=self.launch_log, stderr=subprocess.STDOUT, start_new_session=True)
            self.message = 'Connecting to robot. Keep the arm clear for (105, 30) mm initialization.'
            return dict(success=True, message=self.message)

    def service(self, client, request):
        if not client.service_is_ready():
            return dict(success=False, message='Robot service is unavailable')
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(3):
            return dict(success=False, message='Robot service timed out; check console')
        result = future.result()
        return dict(success=result.success, message=result.message)

    def action(self, name):
        if name == 'connect':
            return self.start_robot()
        if name == 'arm':
            status = self.status()
            if not status['ready']:
                return dict(success=False, message='; '.join(status['reasons']))
            result = self.service(self.arm_client, SetBool.Request(data=True))
            if result['success']:
                self.deadline = time.monotonic()+40
            self.message = result['message']
            return result
        if name == 'stop':
            self.deadline = None
            result = self.service(self.stop_client, Trigger.Request())
            self.message = result['message']
            return result
        if name == 'record':
            with self.mutex:
                if self.writer is not None:
                    return dict(success=False, message='Camera recording already running')
                if time.monotonic()-self.sensor_received > 1:
                    return dict(success=False, message='Wait for live robot telemetry')
                frame = cv2.imread('/tmp/rm_latest.jpg')
                if frame is None:
                    return dict(success=False, message='No camera frame yet')
                path = self.folder / ('camera_'+datetime.now().strftime('%H%M%S')+'.mp4')
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), 10, (w, h))
                if not writer.isOpened():
                    return dict(success=False, message='Could not open MP4 encoder')
                self.writer = writer
                self.record_started = time.monotonic()
                self.recorded_frames = 0
                self.message = 'Recording camera for 45 seconds. Film the robot externally with your phone too.'
                return dict(success=True, message=self.message)
        return dict(success=False, message='Unknown action')

    def tick(self):
        now = time.monotonic()
        if self.deadline and (now >= self.deadline or now-self.last_browser > 3):
            self.deadline = None
            self.message = 'Automatic stop: 40-second run limit or browser disconnected'
            if self.stop_client.service_is_ready():
                self.stop_client.call_async(Trigger.Request())
        with self.mutex:
            if self.writer is not None:
                if now-self.record_started >= 45:
                    self.writer.release()
                    self.writer = None
                    self.message = 'Camera recording saved. Review the video and measured stopping distance.'
                else:
                    frame = cv2.imread('/tmp/rm_latest.jpg')
                    if frame is not None:
                        state = self.applied.get('state', 'OFFLINE')
                        label = f'{now-self.record_started:.1f}s | {state} | camera feed | v={self.applied.get("vx",0):.3f} m/s'
                        cv2.rectangle(frame, (0, 0), (frame.shape[1], 36), (0, 0, 0), -1)
                        cv2.putText(frame, label, (12, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, (255,255,255), 1)
                        self.writer.write(frame)
                        self.recorded_frames += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true', help='check setup without connecting or moving robot')
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    cv2.setNumThreads(2)
    cfg = yaml.safe_load(CONFIG.read_text())
    if args.check:
        print(json.dumps(dict(ros_available=True, config_exists=CONFIG.exists(),
                              page_exists=PAGE.exists(), calibration_verified=cfg['calibration_verified'])))
        return
    stamp = datetime.now(ZoneInfo('Asia/Riyadh')).strftime('%Y%m%d_%H%M%S_%f')
    folder = ROOT/'data'/('hardware_'+stamp)
    folder.mkdir(parents=True)
    token = secrets.token_urlsafe(24)
    rclpy.init()
    panel = Panel(folder)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, code, data, content_type='application/json'):
            body = data if isinstance(data, bytes) else json.dumps(data).encode()
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/':
                self.reply(200, PAGE.read_text().replace('__TOKEN__', token).encode(), 'text/html; charset=utf-8')
            elif path == '/api/status':
                panel.last_browser = time.monotonic()
                self.reply(200, panel.status())
            elif path == '/frame.jpg':
                image_path = Path('/tmp/rm_latest.jpg')
                self.reply(200 if image_path.exists() else 404,
                           image_path.read_bytes() if image_path.exists() else b'', 'image/jpeg')
            elif path == '/api/log':
                log_path = panel.folder/'launch.log'
                self.reply(200, dict(text=log_path.read_text(errors='replace')[-6000:] if log_path.exists() else ''))
            else:
                self.reply(404, dict(error='Not found'))

        def do_POST(self):
            if self.headers.get('X-Control-Token') != token:
                return self.reply(403, dict(error='Invalid control token'))
            result = panel.action(self.path.removeprefix('/api/'))
            self.reply(200, result)

    server = ThreadingHTTPServer(('127.0.0.1', 8765), Handler)
    panel.start_robot()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print('Control panel: http://localhost:8765', flush=True)
    print('Output: '+str(folder), flush=True)
    print('The chassis starts disarmed. Ctrl+C shuts down and commands a stop.', flush=True)
    if not args.no_browser:
        subprocess.Popen(['powershell.exe', '-NoProfile', '-Command',
                          'Start-Process http://localhost:8765'], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    try:
        rclpy.spin(panel)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        if panel.writer is not None:
            panel.writer.release()
        if panel.launch is not None and panel.launch.poll() is None:
            os.killpg(panel.launch.pid, signal.SIGINT)
            try:
                panel.launch.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(panel.launch.pid, signal.SIGTERM)
                panel.launch.wait(timeout=5)
        if panel.launch_log is not None:
            panel.launch_log.close()
        panel.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
