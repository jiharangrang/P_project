"""Phase 1: ROI + HSV 기반 라인 추종 주행 루프."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2

from raspbot.control import PIDController, VehicleController
from raspbot.hardware import Camera, CameraConfig, RaspbotHardware
from raspbot.perception import (
    analyze_histogram,
    apply_roi_overlay,
    calculate_roi_points,
    compute_lane_error,
    detect_road_lines,
    visualize_binary_debug,
    warp_perspective,
)
from raspbot.utils import FpsTimer, load_config


def parse_args():
    parser = argparse.ArgumentParser(description="Phase1 라인 기반 자율주행 루프")
    parser.add_argument(
        "--config",
        type=str,
        default=str(Path(__file__).resolve().parents[2] / "configs" / "phase1_pid.yaml"),
        help="YAML 설정 파일 경로",
    )
    parser.add_argument(
        "--mock-hw",
        action="store_true",
        help="실기 없이 모터/서보/LED 호출을 콘솔 로그로 대체",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="OpenCV 윈도우를 띄우지 않고 동작",
    )
    return parser.parse_args()


def build_camera(cfg) -> Camera:
    camera_cfg = CameraConfig(
        index=cfg.get("index", 0),
        resolution=tuple(cfg.get("resolution", (320, 240))),
        brightness=cfg.get("brightness", 0),
        contrast=cfg.get("contrast", 0),
        saturation=cfg.get("saturation", 50),
        exposure=cfg.get("exposure", 100),
        gain=cfg.get("gain", 0),
    )
    return Camera(camera_cfg)


def run(cfg, args) -> None:
    hardware_cfg = cfg.get("hardware", {})
    perception_cfg = cfg.get("perception", {})
    control_cfg = cfg.get("control", {})
    runtime_cfg = cfg.get("runtime", {})

    show_windows = runtime_cfg.get("show_windows", True) and not args.headless

    hardware = RaspbotHardware(
        use_led=hardware_cfg.get("use_led", True),
        use_beep=hardware_cfg.get("use_beep", True),
        led_on_start=hardware_cfg.get("led_on_start", True),
        beep_on_start=hardware_cfg.get("beep_on_start", True),
        servo_defaults=tuple(hardware_cfg.get("servo_defaults", (70, 10))),
        servo_neutral=tuple(hardware_cfg.get("servo_neutral", (90, 25))),
        led_mode=hardware_cfg.get("led_mode", 2),
        enable_mock=args.mock_hw,
    )

    camera = build_camera(hardware_cfg.get("camera", {}))

    pid = PIDController(
        kp=float(control_cfg.get("kp", 0.9)),
        ki=float(control_cfg.get("ki", 0.0)),
        kd=float(control_cfg.get("kd", 0.05)),
        output_limits=(
            -float(control_cfg.get("steer_limit", 80)),
            float(control_cfg.get("steer_limit", 80)),
        ),
    )

    controller = VehicleController(
        hardware=hardware,
        base_speed=int(control_cfg.get("base_speed", 40)),
        speed_limit=int(control_cfg.get("speed_limit", 120)),
        steer_scale=float(control_cfg.get("steer_scale", 1.0)),
        deadband=float(control_cfg.get("steer_deadband", 0.0)),
    )

    roi_top = perception_cfg.get("roi_top", 871)
    roi_bottom = perception_cfg.get("roi_bottom", 946)
    detect_value = perception_cfg.get("detect_value", 120)
    ipm_resolution = tuple(perception_cfg.get("ipm_resolution", (320, 240)))

    fps_timer = FpsTimer(window=int(runtime_cfg.get("fps_window", 15)))

    last_time = time.perf_counter()

    print("=== Phase 1: 라인 추종 주행 시작 ===")
    print("ESC 또는 q 입력 시 종료됩니다.")

    try:
        while True:
            frame = camera.read()
            height, width = frame.shape[:2]

            pts_src, top_y, bottom_y = calculate_roi_points(width, height, roi_top, roi_bottom)
            frame_with_roi = apply_roi_overlay(frame, pts_src, width, height, top_y, bottom_y)

            warped, _ = warp_perspective(frame, pts_src, ipm_resolution)
            gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
            binary = detect_road_lines(warped, gray, detect_value)

            error_norm, histogram, centroid_x, hist_stats = compute_lane_error(binary)
            now = time.perf_counter()
            dt = now - last_time
            last_time = now

            if error_norm is None:
                direction = "STOP"
                steering_output = 0.0
                controller.stop()
                if runtime_cfg.get("fail_safe_beep", True):
                    hardware.beep(0.05)
            else:
                steering_output = pid.update(error_norm, dt)
                _, _, direction = controller.drive(steering_output)

            fps = fps_timer.lap()

            if runtime_cfg.get("print_debug", False):
                if hist_stats:
                    left_sum, center_sum, right_sum, left_ratio, center_ratio, right_ratio = hist_stats
                    print(
                        f"err={error_norm if error_norm is not None else 'None'} "
                        f"steer={steering_output:.2f} "
                        f"hist L{left_sum}({left_ratio:.2f}) C{center_sum}({center_ratio:.2f}) R{right_sum}({right_ratio:.2f})"
                    )
                else:
                    print("라인이 보이지 않습니다. STOP 상태 유지.")

            if show_windows:
                debug_binary = visualize_binary_debug(
                    binary, direction, hist_stats, centroid_x, steering_output, fps
                )
                cv2.imshow("phase1/frame", frame_with_roi)
                cv2.imshow("phase1/binary", debug_binary)

                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
            else:
                time.sleep(runtime_cfg.get("headless_delay", 0.01))

    finally:
        controller.stop()
        camera.release()
        hardware.cleanup()
        if show_windows:
            cv2.destroyAllWindows()
        print("정상 종료되었습니다.")


def main():
    args = parse_args()
    cfg = load_config(args.config)
    run(cfg, args)


if __name__ == "__main__":
    sys.exit(main())
