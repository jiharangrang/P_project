"""라인 검출 및 에러 계산."""

from __future__ import annotations

from typing import List, Optional, Tuple

import cv2
import numpy as np


def detect_road_lines(color_frame, gray_frame, detect_value: int) -> np.ndarray:
    """빨간색/엷은 회색 차선을 검출한 이진 이미지를 반환."""
    hsv_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    mask_red1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)

    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])
    mask_red2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    threshold_gray = max(detect_value - 30, 80)
    _, mask_gray = cv2.threshold(gray_frame, threshold_gray, 255, cv2.THRESH_BINARY)

    dark_threshold = 50
    _, mask_dark = cv2.threshold(gray_frame, dark_threshold, 255, cv2.THRESH_BINARY)
    mask_gray = cv2.bitwise_and(mask_gray, mask_dark)

    mask_lines = cv2.bitwise_or(mask_red, mask_gray)

    kernel = np.ones((3, 3), np.uint8)
    mask_lines = cv2.morphologyEx(mask_lines, cv2.MORPH_CLOSE, kernel)
    mask_lines = cv2.morphologyEx(mask_lines, cv2.MORPH_OPEN, kernel)
    return mask_lines


def analyze_histogram(histogram: np.ndarray):
    length = len(histogram)
    left_end = length // 3
    right_start = 2 * length // 3

    left_sum = int(np.sum(histogram[:left_end]))
    center_sum = int(np.sum(histogram[left_end:right_start]))
    right_sum = int(np.sum(histogram[right_start:]))

    left_ratio = left_sum / (left_end * 255) if left_end > 0 else 0
    center_ratio = (
        center_sum / ((right_start - left_end) * 255)
        if (right_start - left_end) > 0
        else 0
    )
    right_ratio = (
        right_sum / ((length - right_start) * 255) if (length - right_start) > 0 else 0
    )

    return left_sum, center_sum, right_sum, left_ratio, center_ratio, right_ratio


def compute_lane_error(
    binary_frame: np.ndarray,
) -> Tuple[Optional[float], Optional[np.ndarray], Optional[int], Optional[tuple]]:
    """
    도로(검정 영역)를 기준으로 중심 오차를 계산.

    Returns:
        error_norm: (-1~1) 정규화된 중심 오차. 좌측이 음수, 우측이 양수.
        histogram: 수평 합산 결과
        centroid_x: 도로 영역 무게중심 x 좌표
        histogram_stats: (left_sum, center_sum, right_sum, left_ratio, center_ratio, right_ratio)
    """
    height, width = binary_frame.shape[:2]
    road_mask = cv2.bitwise_not(binary_frame)
    moments = cv2.moments(road_mask)

    if moments["m00"] < 1e-3:
        return None, None, None, None

    centroid_x = int(moments["m10"] / moments["m00"])
    error_px = centroid_x - (width // 2)
    error_norm = error_px / (width / 2)

    histogram = np.sum(binary_frame, axis=0)
    histogram_stats = analyze_histogram(histogram)
    return error_norm, histogram, centroid_x, histogram_stats


def estimate_heading(
    binary_frame: np.ndarray,
    bands: Tuple[Tuple[float, float], ...] = ((0.15, 0.3), (0.45, 0.6), (0.75, 0.9)),
) -> Tuple[Optional[float], List[Tuple[int, int]]]:
    """
    여러 높이 구간에서 도로 중심을 추정해 진행 방향 기울기를 계산.

    Returns:
        slope_norm: 하단 대비 상단 중심 이동량을 (-1~1)로 정규화한 값. 음수=좌, 양수=우.
        centers: 각 구간에서 구한 (x, y) 중심 좌표 리스트
    """
    h, w = binary_frame.shape[:2]
    road_mask = cv2.bitwise_not(binary_frame)
    centers: List[Tuple[int, int]] = []

    for band in bands:
        y0 = int(band[0] * h)
        y1 = int(band[1] * h)
        y0 = max(0, min(h - 1, y0))
        y1 = max(0, min(h, y1))
        if y1 <= y0:
            continue
        band_mask = road_mask[y0:y1, :]
        m = cv2.moments(band_mask)
        if m["m00"] < 1e-3:
            continue
        cx = int(m["m10"] / m["m00"])
        cy = (y0 + y1) // 2
        centers.append((cx, cy))

    if len(centers) < 2:
        return None, centers

    bottom_x, _ = centers[-1]
    top_x, _ = centers[0]
    slope_px = top_x - bottom_x
    slope_norm = slope_px / (w / 2)
    slope_norm = float(max(-1.0, min(1.0, slope_norm)))
    return slope_norm, centers


def edge_occupancy(
    binary_frame: np.ndarray,
    width_px: int = 15,
    height_range: Tuple[float, float] = (0.5, 1.0),
    smooth_alpha: float = 0.2,
    prev_value: Optional[float] = None,
) -> Tuple[float, float, float]:
    """
    좌/우 끝단 검정 비율을 계산해 보조 조향 신호로 사용.

    Returns:
        edge_diff: (-1~1) 좌측-우측 검정 비율 차. 양수=좌측 검정(공간) 많음 → 왼쪽으로 기울기.
        left_black_ratio: 좌측 구간 검정 비율
        right_black_ratio: 우측 구간 검정 비율
    """
    h, w = binary_frame.shape[:2]
    y0 = int(height_range[0] * h)
    y1 = int(height_range[1] * h)
    y0 = max(0, min(h, y0))
    y1 = max(0, min(h, y1))
    if y1 <= y0:
        return 0.0, 0.0, 0.0

    road_mask = cv2.bitwise_not(binary_frame[y0:y1, :])

    width_px = max(1, min(w // 3, width_px))
    left_slice = road_mask[:, :width_px]
    right_slice = road_mask[:, -width_px:]

    total_pixels = left_slice.size
    left_black_ratio = float(np.sum(left_slice > 0) / total_pixels) if total_pixels > 0 else 0.0
    right_black_ratio = float(np.sum(right_slice > 0) / total_pixels) if total_pixels > 0 else 0.0

    diff = left_black_ratio - right_black_ratio
    denom = max(left_black_ratio + right_black_ratio, 1e-6)
    edge_diff = diff / denom
    edge_diff = float(max(-1.0, min(1.0, edge_diff)))

    if prev_value is not None:
        edge_diff = smooth_alpha * edge_diff + (1 - smooth_alpha) * prev_value

    return edge_diff, left_black_ratio, right_black_ratio
