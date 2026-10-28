"""AprilTag pose and a latest-frame-only OpenCV receiver."""
import math
import threading
import time
import cv2
import numpy as np
from robomaster import media


def _far_range_parameters():
    """Parameters that tolerate small, mildly blurred markers and uneven light."""
    p = cv2.aruco.DetectorParameters_create()
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 31
    p.adaptiveThreshWinSizeStep = 4
    p.adaptiveThreshConstant = 5
    p.minMarkerPerimeterRate = 0.006
    p.polygonalApproxAccuracyRate = 0.08
    p.minOtsuStdDev = 2.0
    p.perspectiveRemovePixelPerCell = 2
    p.perspectiveRemoveIgnoredMarginPerCell = 0.15
    p.maxErroneousBitsInBorderRate = 0.45
    p.errorCorrectionRate = 0.7
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 3
    return p


def detect_markers_with_fallback(gray, dictionary, parameters=None, target_id=None):
    """Detect markers with progressively stronger, bounded far-range passes.

    The ordinary image is tried first for speed. If the requested marker is not
    found, retry with looser contour/threshold settings, local contrast
    equalization, and a 2x upsample. Upsampling helps the contour and cell
    sampler; it cannot recover detail that the camera did not resolve.
    """
    if gray.ndim != 2:
        raise ValueError('detect_markers_with_fallback expects a grayscale image')
    if parameters is None:
        parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected = cv2.aruco.detectMarkers(
        gray, dictionary, parameters=parameters)
    if ids is not None and (target_id is None or target_id in ids.flatten()):
        return corners, ids, rejected

    # Keep the baseline result if it found a different ID, but continue looking
    # for the requested target instead of returning early.
    best = (corners, ids, rejected)
    fallback = _far_range_parameters()
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    # Construct expensive images only after cheaper passes fail.
    passes = [(lambda: gray, fallback, 1),
              (lambda: enhanced, fallback, 1),
              (lambda: cv2.resize(enhanced, None, fx=2, fy=2,
                                 interpolation=cv2.INTER_CUBIC), fallback, 2)]

    for image, params, scale in passes:
        found_corners, found_ids, found_rejected = cv2.aruco.detectMarkers(
            image(), dictionary, parameters=params)
        if found_ids is not None:
            if scale != 1:
                found_corners = [c.astype(np.float32) / scale
                                 for c in found_corners]
            if target_id is None or target_id in found_ids.flatten():
                return found_corners, found_ids, found_rejected
            if best[1] is None or len(found_ids) > len(best[1]):
                best = (found_corners, found_ids, found_rejected)
    return best


class LatestVideo:
    """SDK LiveView adapter; exactly one decoder, bounded buffering."""
    def __init__(self):
        self.running = False
        self.frame = None
        self.stamp = -1e9
        self.lock = threading.Lock()
        self.decoder = None

    def start_video_stream(self, display, addr, protocol):
        # The SDK's incremental H.264 decoder tolerates streams that begin
        # between parameter sets. OpenCV/FFmpeg intermittently connected after
        # the SPS/PPS and then produced no frames under WSL mirrored networking.
        self.decoder = media.LiveView(None)
        if not self.decoder.start_video_stream(False, addr, protocol):
            return False
        self.running = True
        self.thread = threading.Thread(target=self.grab, daemon=True)
        self.thread.start()
        return True

    def grab(self):
        while self.running:
            frame = self.decoder.read_video_frame(timeout=0.2, strategy='newest')
            if frame is not None:
                with self.lock:
                    self.frame, self.stamp = frame, time.monotonic()
            time.sleep(0.010)  # Required decoder-buffer recycling pause.

    def latest(self):
        with self.lock:
            return self.stamp, None if self.frame is None else self.frame.copy()

    def stop_video_stream(self):
        self.running = False
        if self.decoder is not None:
            self.decoder.stop_video_stream()
        if hasattr(self, 'thread'):
            self.thread.join(timeout=2)
        return True

    def stop(self):
        self.stop_video_stream()


class TagDetector:
    def __init__(self, cfg):
        self.cfg = cfg
        self.K = np.array(cfg['camera_matrix'], float).reshape(3, 3)
        self.D = np.array(cfg['distortion'], float)
        self.R = np.array(cfg['camera_to_body_rotation'], float).reshape(3, 3)
        self.translation = np.array(cfg['camera_to_body_translation_m'], float)
        if not np.allclose(self.R.T@self.R, np.eye(3), atol=1e-5) or np.linalg.det(self.R) < 0.99:
            raise ValueError('Camera rotation must be a proper orthonormal matrix')
        if cfg['tag_size_m'] <= 0 or self.K[0, 0] <= 0 or self.K[1, 1] <= 0:
            raise ValueError('Positive measured tag size and focal lengths are required')
        dictionary = getattr(cv2.aruco, cfg['tag_family'])
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._frames_seen = 0
        self.last_ids = []
        self.last_corners = []
        s = cfg['tag_size_m']/2
        self.points = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])

    def detect(self, frame):
        h, w = frame.shape[:2]
        if [w, h] != self.cfg['image_size']:
            raise ValueError(f'Calibration size mismatch: frame {w}x{h}')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self._frames_seen += 1
        # A full enhanced/upsampled search is relatively expensive. Try it on
        # every third frame after the normal fast pass misses, keeping the
        # detector responsive while still retrying well inside the 0.45 s loss
        # timeout at the driver's 20 Hz tick rate.
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters)
        if (ids is None or 3 not in ids.flatten()) and self._frames_seen % 3 == 0:
            corners, ids, _ = detect_markers_with_fallback(
                gray, self.dictionary, self.parameters, target_id=3)
        self.last_ids = [] if ids is None else [int(i) for i in ids.flatten()]
        self.last_corners = corners
        if ids is None:
            return None
        for c, tag_id in zip(corners, ids.flatten()):
            if int(tag_id) != 3:
                continue
            ok, rvec, tvec = cv2.solvePnP(self.points, c.reshape(4, 2), self.K, self.D,
                                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok or tvec[2, 0] <= 0:
                continue
            projection, _ = cv2.projectPoints(self.points, rvec, tvec, self.K, self.D)
            error = float(np.sqrt(np.mean((projection.reshape(4, 2)-c.reshape(4, 2))**2)))
            if error > self.cfg['max_reprojection_px']:
                continue
            body = self.R@tvec.flatten()+self.translation
            if body[0] <= 0:
                continue
            return dict(id=3, distance=math.hypot(body[0], body[1]),
                        bearing=math.atan2(body[1], body[0]),
                        position_body_m=body.tolist(), reprojection_px=error)
        return None
