"""Exercise the ROS driver without moving or connecting to a real robot."""
import threading
import time
from types import SimpleNamespace

import numpy as np
import rclpy
import yaml
from std_srvs.srv import SetBool, Trigger

from rm_flatness_nav import driver


def test_driver_initialization_gate_and_stop_do_not_wait_for_vision(tmp_path, monkeypatch):
    cfg = dict(calibration_verified=False, local_ip='127.0.0.1', robot_ip='',
               conn_type='ap', front_tof_index=0)
    config_path = tmp_path/'hardware.yaml'
    config_path.write_text(yaml.safe_dump(cfg))
    commands, arm_poses = [], []
    detecting = threading.Event()
    release = threading.Event()

    class FakeVideo:
        def __init__(self):
            self.stamp = time.monotonic()
        def latest(self):
            return self.stamp, np.zeros((720, 1280, 3), np.uint8)
        def stop(self):
            pass

    class SlowDetector:
        last_ids = []
        last_corners = []
        def __init__(self, cfg):
            pass
        def detect(self, image):
            detecting.set()
            release.wait(2)
            return None

    class FakeSDK:
        def __init__(self):
            self.chassis = SimpleNamespace(
                drive_speed=lambda **kw: commands.append(kw) or True,
                sub_esc=lambda freq, callback: callback([0]*4) or True,
                sub_velocity=lambda freq, callback: callback([0]*6) or True,
                unsub_esc=lambda: None, unsub_velocity=lambda: None)
            self.sensor = SimpleNamespace(
                sub_distance=lambda freq, callback: callback([0]*4) or True,
                unsub_distance=lambda: None)
            self.robotic_arm = SimpleNamespace(moveto=self.moveto)
            self.camera = SimpleNamespace(start_video_stream=lambda **kw: True)
        def moveto(self, **kw):
            arm_poses.append(kw)
            return SimpleNamespace(wait_for_completed=lambda timeout: True)
        def initialize(self, **kw):
            return True
        def close(self):
            pass

    monkeypatch.setattr(driver.robot, 'Robot', FakeSDK)
    monkeypatch.setattr(driver, 'LatestVideo', FakeVideo)
    monkeypatch.setattr(driver, 'TagDetector', SlowDetector)
    # Test should not overwrite the live dashboard image.
    monkeypatch.setattr(driver.cv2, 'imwrite', lambda *args: True)
    monkeypatch.setattr(driver.os, 'replace', lambda *args: None)
    rclpy.init(args=['--ros-args', '-p', f'config_file:={config_path}'])
    node = None
    try:
        node = driver.Driver()
        assert arm_poses == [dict(x=105, y=30)]
        assert detecting.wait(1)
        response = node.arm(SetBool.Request(data=True), SetBool.Response())
        assert not response.success
        assert not node.armed
        # Even if an upstream command requests motion, a blocked image worker
        # cannot delay the driver timer, and a missing tag blocks translation.
        node.armed = True
        node.command = dict(stamp=time.monotonic(), vx=.1, wz=0, state='CHASING')
        started = time.monotonic()
        node.tick()
        assert time.monotonic()-started < .2
        assert commands[-1]['x'] == 0
        response = node.stop_service(Trigger.Request(), Trigger.Response())
        assert response.success and node.latched_stop and not node.armed
        assert all(c['x'] == c['y'] == c['z'] == 0 for c in commands)
    finally:
        release.set()
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_driver_survives_missing_zero_command_ack_and_inhibits_arm(tmp_path, monkeypatch):
    cfg = dict(calibration_verified=True, local_ip='127.0.0.1', robot_ip='',
               conn_type='ap', front_tof_index=0)
    config_path = tmp_path/'hardware.yaml'
    config_path.write_text(yaml.safe_dump(cfg))

    class FakeVideo:
        stamp = time.monotonic()
        def latest(self):
            return self.stamp, np.zeros((720, 1280, 3), np.uint8)
        def stop(self):
            pass

    class FastDetector:
        last_ids = []
        last_corners = []
        def __init__(self, cfg):
            pass
        def detect(self, image):
            return None

    class FakeSDK:
        def __init__(self):
            self.chassis = SimpleNamespace(
                drive_speed=lambda **kw: False,
                sub_esc=lambda freq, callback: callback([0]*4) or True,
                sub_velocity=lambda freq, callback: callback([0]*6) or True,
                unsub_esc=lambda: None, unsub_velocity=lambda: None)
            self.sensor = SimpleNamespace(
                sub_distance=lambda freq, callback: callback([0]*4) or True,
                unsub_distance=lambda: None)
            self.robotic_arm = SimpleNamespace(
                moveto=lambda **kw: SimpleNamespace(wait_for_completed=lambda timeout: True))
            self.camera = SimpleNamespace(start_video_stream=lambda **kw: True)
        def initialize(self, **kw):
            return True
        def close(self):
            pass

    monkeypatch.setattr(driver.robot, 'Robot', FakeSDK)
    monkeypatch.setattr(driver, 'LatestVideo', FakeVideo)
    monkeypatch.setattr(driver, 'TagDetector', FastDetector)
    monkeypatch.setattr(driver.cv2, 'imwrite', lambda *args: True)
    monkeypatch.setattr(driver.os, 'replace', lambda *args: None)
    rclpy.init(args=['--ros-args', '-p', f'config_file:={config_path}'])
    node = None
    try:
        node = driver.Driver()
        node.tick()
        assert not node.last_command_ack
        assert not node.armed
        assert not node.latched_stop
        response = node.arm(SetBool.Request(data=True), SetBool.Response())
        assert not response.success
        assert not node.armed
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
