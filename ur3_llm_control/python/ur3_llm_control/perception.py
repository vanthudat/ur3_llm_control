"""Color segmentation + calibrated RGB-D backprojection for the fixed side camera."""
import math
import cv2
import numpy as np

RANGES = {
    'red_cube': [(0, 10), (170, 179)], 'yellow_cube': [(20, 38)],
    'blue_cube': [(100, 130)], 'green_cube': [(42, 85)], 'purple_cube': [(135, 165)]}

TABLE_CAMERA_POSITION = (0.35, -0.75, 1.35)
TABLE_CAMERA_PITCH = 0.681
TABLE_CAMERA_YAW = math.pi / 2.0


def detect_blocks(rgb, depth, intrinsics, camera_position=TABLE_CAMERA_POSITION,
                  pitch=TABLE_CAMERA_PITCH, yaw=TABLE_CAMERA_YAW):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    fx, fy, cx, cy = intrinsics
    cp, sp, cyaw, syaw = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cyaw*cp, -syaw, cyaw*sp],
                         [syaw*cp, cyaw, syaw*sp], [-sp, 0, cp]])
    objects = {}
    for name, ranges in RANGES.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, (lo, 100, 55), (hi, 255, 255))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        candidates = []
        for label in range(1, count):
            if stats[label, cv2.CC_STAT_AREA] < 35:
                continue
            component = labels == label
            ys, xs = np.where(component & np.isfinite(depth) & (depth > 0.1))
            if len(xs) < 25:
                continue
            d = depth[ys, xs]
            local = np.stack([d, -(xs-cx)*d/fx, -(ys-cy)*d/fy], axis=1)
            points = local @ rotation.T + np.array(camera_position)
            top_z = float(np.percentile(points[:, 2], 90))
            # Keep top-face points, excluding visible side faces and colored zone markers.
            top = points[np.abs(points[:, 2] - top_z) < 0.004]
            if len(top) < 15:
                continue
            center = np.median(top, axis=0)
            z = float(center[2]) - 0.020
            if z < 0.730 or z > 1.2:
                continue
            p = [float(center[0]), float(center[1]), z]
            if 0.10 <= p[0] <= 0.58 and abs(p[1]) <= 0.28:
                candidates.append(p)
        # Ambiguous detections are omitted, never guessed.
        if len(candidates) == 1:
            objects[name] = candidates[0]
    return objects


def detect_wrist_blocks(rgb, depth):
    """Detect block colors at close range in the wrist-camera jaw area."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    height, width = hsv.shape[:2]
    if depth.shape != (height, width):
        return {}
    # The wrist camera is aimed into the jaw gap. Exclude image borders where
    # blocks on the table can appear unrelated to the object being carried.
    x0, x1 = int(width * 0.15), int(width * 0.85)
    y0, y1 = int(height * 0.15), int(height * 0.85)
    roi = hsv[y0:y1, x0:x1]
    depth_roi = depth[y0:y1, x0:x1]
    close_range = np.isfinite(depth_roi) & (depth_roi >= 0.04) & (depth_roi <= 0.16)
    visible = {}
    for name, ranges in RANGES.items():
        mask = np.zeros(roi.shape[:2], np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(roi, (lo, 75, 45), (hi, 255, 255))
        mask[~close_range] = 0
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        largest = max((int(stats[i, cv2.CC_STAT_AREA]) for i in range(1, count)), default=0)
        pixels = int(cv2.countNonZero(mask))
        # Keep a small minimum component for portions of a cube visible between
        # the fingers, while also rejecting isolated colored pixels/noise.
        if largest >= 16 and pixels >= 24:
            visible[name] = {
                'pixels': pixels,
                'depth_m': float(np.median(depth_roi[mask > 0])),
            }
    return visible
