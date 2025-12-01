"""라인 검출 및 에러 계산."""

from __future__ import annotations

from typing import Optional, Tuple

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
