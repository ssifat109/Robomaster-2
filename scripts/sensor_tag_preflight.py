"""Read-only ToF and AprilTag check at the assessment arm pose."""
import argparse
import json
import socket
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from robomaster import robot, config, dds
from rm_flatness_nav.vision import LatestVideo, detect_markers_with_fallback

ROOT = Path('/mnt/d/RB 2')
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--duration', type=float, default=8.0,
                    help='stationary camera check duration in seconds')
args = parser.parse_args()
cfg = yaml.safe_load((ROOT/'rm_flatness_nav/config/hardware.yaml').read_text())
out = ROOT/'evidence/sensor_tag_preflight'
out.mkdir(parents=True, exist_ok=True)
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    s.connect(('192.168.2.1', 20020))
    local_ip = s.getsockname()[0]
    if not local_ip.startswith('192.168.2.'):
        raise SystemExit(
            f'WSL routed RoboMaster traffic through {local_ip}; connect Windows to '
            'the RoboMaster Wi-Fi and restart WSL before retrying')
    config.LOCAL_IP_STR = local_ip

ep, video, distances = robot.Robot(), LatestVideo(), []
subscribed = False
camera_started = False
initialized = False
try:
    if not ep.initialize(conn_type='ap'):
        raise RuntimeError('SDK initialization failed')
    initialized = True
    subscribed = bool(ep.sensor.sub_distance(
        freq=10, callback=lambda d: distances.append(list(d))))
    # SDK callback returns only distances. Retain decoded channel IDs and flags
    # from the same subject so a missing/unbound module can be distinguished.
    tof_subject = ep.dds._publisher[dds.DDS_TOF].subject
    ep.camera._liveview = video
    if not ep.camera.start_video_stream(display=False, resolution='720p'):
        raise RuntimeError('Camera failed')
    camera_started = True
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector_params = cv2.aruco.DetectorParameters_create()
    start, latest_ids, frame_stamps = time.monotonic(), [], set()
    tag3_stamps, tag3_centers, tag3_boxes = [], [], []
    while time.monotonic()-start < args.duration:
        stamp, frame = video.latest()
        if frame is not None:
            frame_stamps.add(stamp)
            raw_frame = frame.copy()
            corners, ids, _ = detect_markers_with_fallback(
                cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), dictionary,
                detector_params, target_id=3)
            latest_ids = [] if ids is None else [int(i) for i in ids.flatten()]
            if ids is not None:
                for c, tag_id in zip(corners, ids.flatten()):
                    if int(tag_id) == 3:
                        pts = c.reshape(4, 2)
                        tag3_stamps.append(stamp)
                        tag3_centers.append(pts.mean(axis=0).tolist())
                        tag3_boxes.append((float(pts[:, 0].max()-pts[:, 0].min()),
                                           float(pts[:, 1].max()-pts[:, 1].min())))
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.imwrite(str(out/'latest.jpg'), frame)
            cv2.imwrite(str(out/'latest_raw.jpg'), raw_frame)
        time.sleep(.10)
    unique_tag_stamps = sorted(set(tag3_stamps))
    gaps = [b-a for a, b in zip(unique_tag_stamps, unique_tag_stamps[1:])]
    result = dict(
        tof_samples=len(distances), latest_tof_mm=distances[-1] if distances else None,
        all_tof_values=sorted(set(x for sample in distances for x in sample)),
        decoded_sensor_ids=tof_subject._cmd_id,
        decoded_sensor_flags=tof_subject._flag,
        decoded_sensor_directions=tof_subject._direct,
        frames_sampled=len(frame_stamps),
        tag3_detected_frames=len(unique_tag_stamps),
        tag3_detection_fraction=(len(unique_tag_stamps)/len(frame_stamps)
                                 if frame_stamps else 0.0),
        tag3_max_detection_gap_s=max(gaps) if gaps else None,
        tag3_median_center_px=(np.median(tag3_centers, axis=0).tolist()
                               if tag3_centers else None),
        tag3_median_bbox_px=(np.median(tag3_boxes, axis=0).tolist()
                             if tag3_boxes else None),
        latest_detected_tag_ids=latest_ids)
    (out/'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
finally:
    video.stop()
    if camera_started:
        try:
            ep.camera.stop_video_stream()
        except Exception:
            pass
    if subscribed:
        try:
            ep.sensor.unsub_distance()
        except Exception:
            pass
    if initialized:
        try:
            ep.close()
        except Exception:
            pass
    else:
        # Some RoboMaster SDK releases leave a None worker thread after a
        # connection timeout; their destructor then raises while cleaning up.
        try:
            if getattr(ep, '_client', None) is not None:
                ep._client.stop = lambda: None
            ep.close = lambda: None
        except Exception:
            pass
