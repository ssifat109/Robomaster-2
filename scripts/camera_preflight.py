"""Camera/ToF check; optional authorized arm setup, never chassis motion."""
import argparse
import json
import socket
import time
from pathlib import Path
import cv2
from robomaster import robot, config
from rm_flatness_nav.vision import LatestVideo

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--setup-arm', action='store_true')
p.add_argument('--duration', type=float, default=6)
a = p.parse_args()
out = Path('/mnt/d/RB 2/evidence') / ('arm_preflight' if a.setup_arm else 'camera_preflight')
out.mkdir(parents=True, exist_ok=True)
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.connect(('192.168.2.1', 20020))
    config.LOCAL_IP_STR = sock.getsockname()[0]
ep = robot.Robot()
video = LatestVideo()
tof = []
try:
    if not ep.initialize(conn_type='ap'):
        raise RuntimeError('SDK connection failed')
    if a.setup_arm:
        ep.chassis.drive_speed(x=0, y=0, z=0)
        print('Moving arm to X=105 mm, Y=30 mm; chassis commanded zero.', flush=True)
        action = ep.robotic_arm.moveto(x=105, y=30)
        if not action.wait_for_completed(timeout=10):
            raise RuntimeError('Arm action did not complete')
        print('Arm action completed.', flush=True)
    ep.sensor.sub_distance(freq=10, callback=lambda x: tof.append(list(x)))
    ep.camera._liveview = video
    if not ep.camera.start_video_stream(display=False, resolution='720p'):
        raise RuntimeError('Camera could not start')
    start = time.monotonic()
    stamps = set()
    while time.monotonic()-start < a.duration:
        stamp, frame = video.latest()
        if frame is not None:
            stamps.add(stamp)
            cv2.imwrite(str(out/'latest.jpg'), frame)
        time.sleep(.1)
    result = dict(unique_frames_sampled=len(stamps), tof_samples=len(tof),
                  latest_tof_mm=tof[-1] if tof else None,
                  frame_shape=list(frame.shape) if frame is not None else None)
    (out/'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    if not stamps:
        raise RuntimeError('No camera frames')
finally:
    video.stop()
    try:
        ep.sensor.unsub_distance()
    finally:
        ep.close()
