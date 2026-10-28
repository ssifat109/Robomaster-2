"""Check the last stationary image and retain detected calibration views."""
import json
from pathlib import Path
from datetime import datetime
import cv2

root = Path('/mnt/d/RB 2')
path = root/'evidence/camera_preflight/latest.jpg'
frame = cv2.imread(str(path))
if frame is None:
    raise RuntimeError('No snapshot')
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
ok, corners = cv2.findChessboardCornersSB(gray, (9, 6))
result = dict(found=bool(ok), inner_corners=0 if corners is None else len(corners))
if ok:
    folder = root/'data/calibration'
    folder.mkdir(parents=True, exist_ok=True)
    target = folder/(datetime.now().strftime('view_%Y%m%d_%H%M%S')+'.jpg')
    cv2.imwrite(str(target), frame)
    cv2.drawChessboardCorners(frame, (9, 6), corners, ok)
    result['saved'] = str(target)
cv2.imwrite(str(path.with_name('board_check.jpg')), frame)
print(json.dumps(result))
