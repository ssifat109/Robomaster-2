import cv2
import numpy as np
from rm_flatness_nav.vision import TagDetector, detect_markers_with_fallback


def config():
    return dict(camera_matrix=[800, 0, 640, 0, 800, 360, 0, 0, 1],
                distortion=[0]*5, camera_to_body_rotation=[0, 0, 1, -1, 0, 0, 0, -1, 0],
                camera_to_body_translation_m=[0, 0, 0], tag_size_m=.04,
                image_size=[1280, 720], tag_family='DICT_APRILTAG_36h11', max_reprojection_px=2.)


def test_actual_tag_image_id_and_metric_pose():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    frame = np.full((720, 1280, 3), 255, np.uint8)
    # 160 pixels at focal length 800, for a 40 mm tag => 0.20 m.
    marker = cv2.aruco.drawMarker(d, 3, 160)
    frame[280:440, 560:720] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    result = TagDetector(config()).detect(frame)
    assert result['id'] == 3
    assert abs(result['distance']-.2) < .003
    assert abs(result['bearing']) < .003


def test_wrong_id_ignored():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    frame = np.full((720, 1280, 3), 255, np.uint8)
    frame[280:440, 560:720] = cv2.cvtColor(cv2.aruco.drawMarker(d, 1, 160), cv2.COLOR_GRAY2BGR)
    assert TagDetector(config()).detect(frame) is None


def test_far_small_blurred_tag_found_by_fallback():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.drawMarker(dictionary, 3, 200)
    src = np.array([[0, 0], [199, 0], [199, 199], [0, 199]], np.float32)
    # Approximate a small, floor-plane tag near the top of a 720p frame.
    dst = np.array([[610, 80], [650, 80], [650, 92], [610, 92]], np.float32)
    homography = cv2.getPerspectiveTransform(src, dst)
    image = cv2.warpPerspective(marker, homography, (1280, 720), borderValue=255)
    image = cv2.GaussianBlur(image, (3, 3), 0.5)
    rng = np.random.default_rng(7)
    image = np.clip(image.astype(np.float32) * 0.75 + 50 +
                    rng.normal(0, 3, image.shape), 0, 255).astype(np.uint8)

    corners, ids, _ = detect_markers_with_fallback(
        image, dictionary, target_id=3)

    assert ids is not None and 3 in ids.flatten()
    decoded = corners[list(ids.flatten()).index(3)].reshape(4, 2)
    assert np.ptp(decoded[:, 0]) < 55
    assert np.ptp(decoded[:, 1]) < 20


def test_requested_id_search_continues_after_other_id():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    image = np.full((720, 1280), 255, np.uint8)
    clear = cv2.aruco.drawMarker(dictionary, 1, 160)
    image[260:420, 200:360] = clear
    small = cv2.aruco.drawMarker(dictionary, 3, 200)
    src = np.array([[0, 0], [199, 0], [199, 199], [0, 199]], np.float32)
    dst = np.array([[900, 80], [940, 80], [940, 92], [900, 92]], np.float32)
    homography = cv2.getPerspectiveTransform(src, dst)
    small = cv2.warpPerspective(small, homography, (1280, 720), borderValue=255)
    small = cv2.GaussianBlur(small, (3, 3), 0.5)
    image = np.minimum(image, small)

    _, ids, _ = detect_markers_with_fallback(image, dictionary, target_id=3)

    assert ids is not None and 3 in ids.flatten()
