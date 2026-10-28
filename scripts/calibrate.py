"""Calibrate intrinsics from checkerboard images or extrinsics from a surveyed tag.

Intrinsics: --images calibration/*.jpg --config path.yaml
Extrinsics: --tag-image frame.jpg --tag-x 1.0 --tag-height 0.08 --config path.yaml
The extrinsic step assumes an upright, unrotated tag facing the aligned robot,
with its centre on body y=0. Distances use the selected chassis reference.
Both steps leave calibration_verified false until independent range checks pass.
"""
import argparse
from pathlib import Path
import cv2
import numpy as np
import yaml

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--config', type=Path, required=True)
p.add_argument('--images', nargs='+')
p.add_argument('--session', action='store_true', help='Use saved views in data/calibration/view_*.jpg')
p.add_argument('--tag-image')
p.add_argument('--tag-x', type=float)
p.add_argument('--tag-height', type=float)
a = p.parse_args()
cfg = yaml.safe_load(a.config.read_text())
if a.session:
    a.images = [str(x) for x in sorted((Path(__file__).resolve().parents[1]/'data/calibration').glob('view_*.jpg'))]
if a.images:
    obj = np.zeros((9*6, 3), np.float32)
    obj[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)*.020
    objects, pixels, size = [], [], None
    for name in a.images:
        frame = cv2.imread(name, cv2.IMREAD_GRAYSCALE)
        if frame is None:
            raise ValueError(f'Cannot read {name}')
        this_size = frame.shape[::-1]
        if size is not None and size != this_size:
            raise ValueError('Mixed image dimensions')
        size = this_size
        ok, corners = cv2.findChessboardCornersSB(frame, (9, 6))
        if ok:
            objects.append(obj)
            pixels.append(corners)
    if len(objects) < 12:
        raise ValueError(f'Only {len(objects)} valid views; need at least 12 varied poses')
    rms, K, D, _, _ = cv2.calibrateCamera(objects, pixels, size, None, None)
    cfg.update(camera_matrix=K.flatten().tolist(), distortion=D.flatten().tolist(),
               image_size=list(size), intrinsic_rms_px=float(rms), calibration_verified=False)
    print(f'Calibration RMS: {rms:.4f} px from {len(objects)} views')
elif a.tag_image:
    if a.tag_x is None or a.tag_height is None or a.tag_x <= .2:
        p.error('Extrinsics need surveyed --tag-x > 0.2 m and --tag-height in metres')
    frame = cv2.imread(a.tag_image)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg['tag_family']))
    corners, ids, _ = cv2.aruco.detectMarkers(frame, dictionary)
    if ids is None or 3 not in ids:
        raise ValueError('Tag 3 not detected')
    s = cfg['tag_size_m']/2
    points = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])
    K = np.array(cfg['camera_matrix']).reshape(3, 3)
    D = np.array(cfg['distortion'])
    c = corners[list(ids.flatten()).index(3)]
    ok, rv, tv = cv2.solvePnP(points, c.reshape(4, 2), K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or tv[2, 0] <= 0:
        raise ValueError('Pose estimation failed')
    camera_from_tag, _ = cv2.Rodrigues(rv)
    body_from_tag = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    body_from_camera = body_from_tag@camera_from_tag.T
    translation = np.array([a.tag_x, 0, a.tag_height])-body_from_camera@tv.flatten()
    cfg.update(camera_to_body_rotation=body_from_camera.flatten().tolist(),
               camera_to_body_translation_m=translation.tolist(), calibration_verified=False)
    print('Estimated body<-camera translation:', translation)
    print('Verify independently at 1.5 m, 0.5 m and 0.2 m before setting calibration_verified.')
else:
    p.error('Choose --images or --tag-image')
backup = a.config.with_suffix('.yaml.bak')
if not backup.exists():
    backup.write_text(a.config.read_text())
a.config.write_text(yaml.safe_dump(cfg, sort_keys=False))
