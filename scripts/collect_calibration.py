"""Collect geometrically distinct checkerboard views from the live EP stream."""
import json
import argparse
import socket
import time
from pathlib import Path
import cv2
import numpy as np
from robomaster import robot, config
from rm_flatness_nav.vision import LatestVideo

ROOT = Path('/mnt/d/RB 2')
OUT = ROOT/'data/calibration'
OUT.mkdir(parents=True, exist_ok=True)
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--target-views', type=int, default=24,
               help='Total distinct views to collect, including saved views (default: 24)')
p.add_argument('--duration', type=float, default=240,
               help='Maximum collection time in seconds (default: 240)')
a = p.parse_args()
if a.target_views < 12:
    p.error('At least 12 total views are required')
# Seed diversity checks with all already saved checkerboard views.
features = []
for file in sorted(OUT.glob('view_*.jpg')):
    im = cv2.imread(str(file), cv2.IMREAD_GRAYSCALE)
    if im is None:
        continue
    ok, corners = cv2.findChessboardCornersSB(im, (9, 6))
    if ok:
        features.append(corners.reshape(-1, 2)/np.array([[im.shape[1], im.shape[0]]]))
saved = 0
video = LatestVideo()
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.connect(('192.168.2.1', 20020))
    config.LOCAL_IP_STR = sock.getsockname()[0]
ep = robot.Robot()
try:
    if not ep.initialize(conn_type='ap'):
        raise RuntimeError('SDK initialization failed')
    ep.camera._liveview = video
    if not ep.camera.start_video_stream(display=False, resolution='720p'):
        raise RuntimeError('Video stream failed')
    print(json.dumps({'ready': True, 'previous_views': len(features),
                      'need': max(0, a.target_views-len(features)),
                      'instruction': 'Keep the board rigid and move it through varied positions, distances, tilts and rotations; keep all corners visible.'}), flush=True)
    deadline = time.monotonic()+a.duration
    last_stamp = -1
    last_saved = 0
    while len(features) < a.target_views and time.monotonic() < deadline:
        stamp, frame = video.latest()
        if frame is None or stamp <= last_stamp or time.monotonic()-last_saved < .65:
            time.sleep(.03)
            continue
        last_stamp = stamp
        ok, corners = cv2.findChessboardCornersSB(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (9, 6))
        if not ok:
            time.sleep(.03)
            continue
        f = corners.reshape(-1, 2)/np.array([[frame.shape[1], frame.shape[0]]])
        # RMS pixel-coordinate displacement normalized to the frame diagonal.
        diversity = min((float(np.sqrt(np.mean(np.sum((f-g)**2, axis=1))))
                         for g in features), default=1.0)
        if diversity < .055:
            time.sleep(.03)
            continue
        name = OUT/f'view_{time.strftime("%Y%m%d_%H%M%S")}_{len(features)+1:02d}.jpg'
        cv2.imwrite(str(name), frame)
        features.append(f)
        last_saved = time.monotonic()
        saved += 1
        print(json.dumps({'saved': len(features), 'file': str(name),
                          'minimum_view_change': round(diversity, 4),
                          'remaining': max(0, a.target_views-len(features))}), flush=True)
    result = {'total_distinct_views': len(features), 'new_views': saved,
              'target_views': a.target_views, 'complete': len(features) >= a.target_views}
    (OUT/'collection.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    if not result['complete']:
        raise RuntimeError('Timed out. More varied board poses are needed.')
finally:
    video.stop()
    ep.close()
