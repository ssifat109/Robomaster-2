"""Estimate body<-camera transform from one aligned, surveyed, upright AprilTag.

Measure tag centre from the chassis-front reference and from the floor. Keep
robot and tag parallel, centred and stationary. This writes a candidate only;
calibration_verified remains false until range checks pass.
"""
import argparse
import socket
import time
from pathlib import Path
import cv2
import numpy as np
import yaml
from robomaster import robot, config
from rm_flatness_nav.vision import LatestVideo, detect_markers_with_fallback

root = Path('/mnt/d/RB 2')
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--tag-id', type=int, default=3, help='AprilTag 36h11 ID used for this calibration')
p.add_argument('--tag-x', type=float, required=True, help='Measured forward distance (m) from front bumper reference to tag plane')
p.add_argument('--tag-height', type=float, required=True, help='Tag centre height (m) above floor')
p.add_argument('--orientation', choices=['front', 'top'], default='front',
               help='Tag +z faces the robot (front) or points upward (top)')
p.add_argument('--wait', type=float, default=3)
a = p.parse_args()
if a.tag_x < .25 or a.tag_height < 0 or a.tag_height > 1:
    p.error('Use surveyed tag x >= 0.25 m and plausible tag centre height')
path = root/'rm_flatness_nav/config/hardware.yaml'
cfg = yaml.safe_load(path.read_text())
if cfg.get('intrinsic_rms_px', 99) > 1.5:
    p.error('Intrinsics are not calibrated well enough')
ep, video = robot.Robot(), LatestVideo()
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.connect(('192.168.2.1', 20020))
    config.LOCAL_IP_STR = sock.getsockname()[0]
try:
    if not ep.initialize(conn_type='ap'):
        raise RuntimeError('SDK initialization failed')
    ep.camera._liveview = video
    if not ep.camera.start_video_stream(display=False, resolution='720p'):
        raise RuntimeError('Video stream failed')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector_params = cv2.aruco.DetectorParameters_create()
    K = np.array(cfg['camera_matrix']).reshape(3, 3)
    D = np.array(cfg['distortion'])
    s = cfg['tag_size_m']/2
    points = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)
    deadline = time.monotonic()+max(3, a.wait)
    pose = None
    while time.monotonic() < deadline:
        _, frame = video.latest()
        if frame is not None:
            corners, ids, _ = detect_markers_with_fallback(
                cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), dictionary, detector_params,
                target_id=a.tag_id)
            if ids is not None and a.tag_id in ids:
                c = corners[list(ids.flatten()).index(a.tag_id)]
                # IPPE returns two plausible planar poses. Select using the known
                # tag orientation and surveyed robot-frame geometry, rather than
                # trusting whichever ambiguous solution solvePnP returns first.
                result = cv2.solvePnPGeneric(points, c.reshape(4, 2), K, D,
                                             flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if result[0]:
                    body_from_tag = (np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)
                                     if a.orientation == 'front' else
                                     np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=float))
                    candidates = []
                    for rv, tv in zip(result[1], result[2]):
                        rot, _ = cv2.Rodrigues(rv)
                        err = cv2.projectPoints(points, rv, tv, K, D)[0].reshape(4, 2)-c.reshape(4, 2)
                        rms = float(np.sqrt(np.mean(err**2)))
                        tag_in_camera = tv.flatten()
                        # Camera center in body frame, assuming the surveyed tag pose.
                        camera_position_in_tag = -rot.T@tag_in_camera
                        camera_position_body = np.array([a.tag_x, 0, a.tag_height])+body_from_tag@camera_position_in_tag
                        body_from_camera = body_from_tag@rot.T
                        camera_forward = body_from_camera[:, 2]
                        to_tag = np.array([a.tag_x, 0, a.tag_height])-camera_position_body
                        norm = np.linalg.norm(to_tag)
                        facing = float(camera_forward@to_tag/norm) if norm else -1.0
                        physically_plausible = (0.03 < camera_position_body[2] < 1.0
                                                and facing > 0.25
                                                and tag_in_camera[2] > 0)
                        if rms <= cfg['max_reprojection_px'] and physically_plausible:
                            candidates.append((rms, rot, tag_in_camera, camera_position_body))
                    if candidates:
                        best = min(candidates, key=lambda item: item[0])
                        pose = best[1], best[2], best[0]
        time.sleep(.05)
    if pose is None:
        raise RuntimeError(f'Could not stably detect AprilTag {a.tag_id}. Center it, face it toward the robot and retry.')
    camera_from_tag, camera_to_tag, rms = pose
    # Tag axes follow OpenCV IPPE square corner order. Front-face tags have +z
    # toward the robot; top-face tags have +z upward. +x is body-right.
    if a.orientation == 'front':
        body_from_tag = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)
    else:
        body_from_tag = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=float)
    body_from_camera = body_from_tag@camera_from_tag.T
    camera_position_in_tag = -camera_from_tag.T@camera_to_tag
    tag_position_body = np.array([a.tag_x, 0, a.tag_height])
    camera_position_body = tag_position_body+body_from_tag@camera_position_in_tag
    cfg.update(camera_to_body_rotation=body_from_camera.flatten().tolist(),
               camera_to_body_translation_m=camera_position_body.tolist(),
               calibration_verified=False, extrinsic_tag_x_m=a.tag_x,
               extrinsic_tag_height_m=a.tag_height, extrinsic_orientation=a.orientation,
               extrinsic_reprojection_px=rms)
    backup = path.with_suffix('.yaml.extrinsics-backup')
    if not backup.exists():
        backup.write_text(path.read_text())
    path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    print(yaml.safe_dump(dict(camera_to_body_rotation=cfg['camera_to_body_rotation'],
                              camera_to_body_translation_m=cfg['camera_to_body_translation_m'],
                              reprojection_px=rms, calibration_verified=False), sort_keys=False))
finally:
    video.stop()
    ep.close()
