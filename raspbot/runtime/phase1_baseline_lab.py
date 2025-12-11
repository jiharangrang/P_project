"""Phase 1: ROI + Lab 기반 라인 추종 주행 루프."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2

from raspbot.control import PIDController, VehicleController
from raspbot.hardware import Camera, CameraConfig, RaspbotHardware
from raspbot.perception import apply_roi_overlay, calculate_roi_points, visualize_binary_debug, warp_perspective
from raspbot.perception.lane_detection_lab import compute_lane_error, detect_road_lines, estimate_heading
from raspbot.utils import FpsTimer, load_config


CONTROL_WINDOW = "phase1/controls"


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


def create_trackbars(perception_cfg, control_cfg, hardware_cfg, runtime_cfg) -> None:
    """실시간 조정을 위한 트랙바 UI 생성."""
    cv2.namedWindow(CONTROL_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROL_WINDOW, 420, 480)

    # ROI 및 바이너리 스레시홀드
    cv2.createTrackbar("roi_top", CONTROL_WINDOW, int(perception_cfg.get("roi_top", 871)), 1000, lambda x: None)
    cv2.createTrackbar("roi_bottom", CONTROL_WINDOW, int(perception_cfg.get("roi_bottom", 946)), 1000, lambda x: None)
    cv2.createTrackbar("detect_value", CONTROL_WINDOW, int(perception_cfg.get("detect_value", 120)), 255, lambda x: None)
    cv2.createTrackbar(
        "lab_red_l_min",
        CONTROL_WINDOW,
        int(perception_cfg.get("lab_red_l_min", 30)),
        255,
        lambda x: None,
    )
    cv2.createTrackbar(
        "lab_red_a_min",
        CONTROL_WINDOW,
        int(perception_cfg.get("lab_red_a_min", 150)),
        255,
        lambda x: None,
    )
    cv2.createTrackbar(
        "lab_red_b_min",
        CONTROL_WINDOW,
        int(perception_cfg.get("lab_red_b_min", 140)),
        255,
        lambda x: None,
    )
    cv2.createTrackbar(
        "lab_gray_a_dev",
        CONTROL_WINDOW,
        int(perception_cfg.get("lab_gray_a_dev", 15)),
        128,
        lambda x: None,
    )
    cv2.createTrackbar(
        "lab_gray_b_dev",
        CONTROL_WINDOW,
        int(perception_cfg.get("lab_gray_b_dev", 15)),
        128,
        lambda x: None,
    )

    # PID 및 주행 속도
    cv2.createTrackbar(
        "pid_kp_x100",
        CONTROL_WINDOW,
        int(float(control_cfg.get("kp", 0.9)) * 100),
        6000,
        lambda x: None,
    )
    cv2.createTrackbar(
        "pid_ki_x100",
        CONTROL_WINDOW,
        int(float(control_cfg.get("ki", 0.0)) * 100),
        200,
        lambda x: None,
    )
    cv2.createTrackbar(
        "pid_kd_x100",
        CONTROL_WINDOW,
        int(float(control_cfg.get("kd", 0.05)) * 100),
        6000,
        lambda x: None,
    )
    cv2.createTrackbar("base_speed", CONTROL_WINDOW, int(control_cfg.get("base_speed", 40)), 255, lambda x: None)
    cv2.createTrackbar(
        "steer_scale_x100",
        CONTROL_WINDOW,
        int(float(control_cfg.get("steer_scale", 1.0)) * 100),
        300,
        lambda x: None,
    )

    # 회전 감지 및 제어 스케일
    turn_cfg = control_cfg.get("turn", {})
    cv2.createTrackbar(
        "turn_slope_thr_x100",
        CONTROL_WINDOW,
        int(float(turn_cfg.get("slope_thresh", 0.25)) * 100),
        300,
        lambda x: None,
    )
    cv2.createTrackbar(
        "turn_offset_thr_x100",
        CONTROL_WINDOW,
        int(float(turn_cfg.get("offset_thresh", 0.3)) * 100),
        300,
        lambda x: None,
    )
    cv2.createTrackbar(
        "turn_speed_scale_x100",
        CONTROL_WINDOW,
        int(float(turn_cfg.get("speed_scale", 0.7)) * 100),
        100,
        lambda x: None,
    )
    cv2.createTrackbar(
        "turn_steer_scale_x100",
        CONTROL_WINDOW,
        int(float(turn_cfg.get("steer_scale", 1.3)) * 100),
        300,
        lambda x: None,
    )

    # heading 가중치
    heading_cfg = control_cfg.get("heading", {})
    cv2.createTrackbar(
        "heading_smooth_x100",
        CONTROL_WINDOW,
        int(heading_cfg.get("smooth_alpha", 0.2) * 100),
        100,
        lambda x: None,
    )
    cv2.createTrackbar(
        "heading_connect_close_px",
        CONTROL_WINDOW,
        int(heading_cfg.get("connect_close_px", 1)),
        20,
        lambda x: None,
    )
    cv2.createTrackbar(
        "heading_merge_gap_px",
        CONTROL_WINDOW,
        int(heading_cfg.get("merge_gap_px", 1)),
        20,
        lambda x: None,
    )
    cv2.createTrackbar(
        "heading_p1_margin_px",
        CONTROL_WINDOW,
        int(heading_cfg.get("p1_margin_px", 1)),
        20,
        lambda x: None,
    )

    # 카메라 설정
    cam_cfg = hardware_cfg.get("camera", {})
    cv2.createTrackbar("brightness", CONTROL_WINDOW, int(cam_cfg.get("brightness", 0)), 200, lambda x: None)
    cv2.createTrackbar("contrast", CONTROL_WINDOW, int(cam_cfg.get("contrast", 0)), 200, lambda x: None)
    cv2.createTrackbar("saturation", CONTROL_WINDOW, int(cam_cfg.get("saturation", 50)), 200, lambda x: None)
    cv2.createTrackbar("exposure", CONTROL_WINDOW, int(cam_cfg.get("exposure", 100)), 300, lambda x: None)
    cv2.createTrackbar("gain", CONTROL_WINDOW, int(cam_cfg.get("gain", 0)), 200, lambda x: None)

    # 서보 각도 (카메라 앵글)
    servo_defaults = hardware_cfg.get("servo_defaults", (70, 10))
    cv2.createTrackbar("servo1_yaw", CONTROL_WINDOW, int(servo_defaults[0]), 180, lambda x: None)
    cv2.createTrackbar(
        "servo2_pitch",
        CONTROL_WINDOW,
        int(servo_defaults[1]),
        int(RaspbotHardware.SERVO_PITCH_LIMIT),
        lambda x: None,
    )


def run(cfg, args) -> None:
    hardware_cfg = cfg.get("hardware", {})
    perception_cfg = cfg.get("perception", {})
    control_cfg = cfg.get("control", {})
    runtime_cfg = cfg.get("runtime", {})

    show_windows = runtime_cfg.get("show_windows", True) and not args.headless
    enable_sliders = runtime_cfg.get("enable_sliders", True) and show_windows

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
    lab_red_l_min = int(perception_cfg.get("lab_red_l_min", 30))
    lab_red_a_min = int(perception_cfg.get("lab_red_a_min", 150))
    lab_red_b_min = int(perception_cfg.get("lab_red_b_min", 140))
    lab_gray_a_dev = int(perception_cfg.get("lab_gray_a_dev", 15))
    lab_gray_b_dev = int(perception_cfg.get("lab_gray_b_dev", 15))
    ipm_resolution = tuple(perception_cfg.get("ipm_resolution", (320, 240)))

    turn_cfg = control_cfg.get("turn", {})
    turn_slope_thresh = float(turn_cfg.get("slope_thresh", 0.25))
    turn_offset_thresh = float(turn_cfg.get("offset_thresh", 0.3))
    turn_speed_scale = float(turn_cfg.get("speed_scale", 0.7))
    turn_steer_scale = float(turn_cfg.get("steer_scale", 1.3))
    heading_cfg = control_cfg.get("heading", {})
    heading_smooth_alpha = float(heading_cfg.get("smooth_alpha", 0.2))
    heading_connect_close_px = int(heading_cfg.get("connect_close_px", 1))
    heading_merge_gap_px = int(heading_cfg.get("merge_gap_px", 1))
    heading_p1_margin_px = int(heading_cfg.get("p1_margin_px", 1))
    heading_prev = 0.0

    if enable_sliders:
        create_trackbars(perception_cfg, control_cfg, hardware_cfg, runtime_cfg)
        last_cam_settings = (
            int(hardware_cfg.get("camera", {}).get("brightness", 0)),
            int(hardware_cfg.get("camera", {}).get("contrast", 0)),
            int(hardware_cfg.get("camera", {}).get("saturation", 50)),
            int(hardware_cfg.get("camera", {}).get("exposure", 100)),
            int(hardware_cfg.get("camera", {}).get("gain", 0)),
        )
        servo_defaults = tuple(hardware_cfg.get("servo_defaults", (70, 10)))
        last_servo_angles = (
            int(servo_defaults[0]),
            int(min(servo_defaults[1], RaspbotHardware.SERVO_PITCH_LIMIT)),
        )

    fps_timer = FpsTimer(window=int(runtime_cfg.get("fps_window", 15)))

    last_time = time.perf_counter()
    motors_enabled = False
    motors_was_enabled = motors_enabled
    controller.stop()

    print("=== Phase 1: 라인 추종 주행 시작 ===")
    print("키 입력:")
    print("  ESC/q : 프로그램 종료")
    print("  s     : 모터 토글 일시정지/재개 (초기 상태: 정지)")
    print("  SPACE : 일시정지 (아무 키나 누르면 재개)")

    try:
        while True:
            if enable_sliders:
                # ROI 및 검출 임계값 업데이트
                roi_top = cv2.getTrackbarPos("roi_top", CONTROL_WINDOW)
                roi_bottom = cv2.getTrackbarPos("roi_bottom", CONTROL_WINDOW)
                detect_value = cv2.getTrackbarPos("detect_value", CONTROL_WINDOW)
                lab_red_l_min = cv2.getTrackbarPos("lab_red_l_min", CONTROL_WINDOW)
                lab_red_a_min = cv2.getTrackbarPos("lab_red_a_min", CONTROL_WINDOW)
                lab_red_b_min = cv2.getTrackbarPos("lab_red_b_min", CONTROL_WINDOW)
                lab_gray_a_dev = cv2.getTrackbarPos("lab_gray_a_dev", CONTROL_WINDOW)
                lab_gray_b_dev = cv2.getTrackbarPos("lab_gray_b_dev", CONTROL_WINDOW)

                # PID/속도 파라미터 업데이트 (trackbar는 정수이므로 스케일링)
                pid.kp = cv2.getTrackbarPos("pid_kp_x100", CONTROL_WINDOW) / 100.0
                pid.ki = cv2.getTrackbarPos("pid_ki_x100", CONTROL_WINDOW) / 100.0
                pid.kd = cv2.getTrackbarPos("pid_kd_x100", CONTROL_WINDOW) / 100.0
                base_speed_slider = cv2.getTrackbarPos("base_speed", CONTROL_WINDOW)
                steer_scale_slider = cv2.getTrackbarPos("steer_scale_x100", CONTROL_WINDOW) / 100.0

                # 카메라 설정 실시간 적용
                brightness = cv2.getTrackbarPos("brightness", CONTROL_WINDOW)
                contrast = cv2.getTrackbarPos("contrast", CONTROL_WINDOW)
                saturation = cv2.getTrackbarPos("saturation", CONTROL_WINDOW)
                exposure = cv2.getTrackbarPos("exposure", CONTROL_WINDOW)
                gain = cv2.getTrackbarPos("gain", CONTROL_WINDOW)
                cam_settings = (brightness, contrast, saturation, exposure, gain)
                if cam_settings != last_cam_settings:
                    camera.apply_settings(
                        brightness=brightness,
                        contrast=contrast,
                        saturation=saturation,
                        exposure=exposure,
                        gain=gain,
                    )
                    last_cam_settings = cam_settings

                # 서보 각도 실시간 적용 (카메라 앵글)
                servo1 = cv2.getTrackbarPos("servo1_yaw", CONTROL_WINDOW)
                servo2 = cv2.getTrackbarPos("servo2_pitch", CONTROL_WINDOW)
                if (servo1, servo2) != last_servo_angles:
                    hardware.set_servo(1, servo1)
                    hardware.set_servo(2, servo2)
                    last_servo_angles = (servo1, servo2)

                # 회전 감지/제어 파라미터 업데이트
                turn_slope_thresh = cv2.getTrackbarPos("turn_slope_thr_x100", CONTROL_WINDOW) / 100.0
                turn_offset_thresh = cv2.getTrackbarPos("turn_offset_thr_x100", CONTROL_WINDOW) / 100.0
                turn_speed_scale = cv2.getTrackbarPos("turn_speed_scale_x100", CONTROL_WINDOW) / 100.0
                turn_steer_scale = cv2.getTrackbarPos("turn_steer_scale_x100", CONTROL_WINDOW) / 100.0
                heading_smooth_alpha = cv2.getTrackbarPos("heading_smooth_x100", CONTROL_WINDOW) / 100.0
                heading_connect_close_px = cv2.getTrackbarPos("heading_connect_close_px", CONTROL_WINDOW)
                heading_merge_gap_px = cv2.getTrackbarPos("heading_merge_gap_px", CONTROL_WINDOW)
                heading_p1_margin_px = cv2.getTrackbarPos("heading_p1_margin_px", CONTROL_WINDOW)
            else:
                # 슬라이더 미사용 시 기본 설정 유지
                base_speed_slider = int(control_cfg.get("base_speed", 40))
                steer_scale_slider = float(control_cfg.get("steer_scale", 1.0))
                lab_red_l_min = int(perception_cfg.get("lab_red_l_min", 30))
                lab_red_a_min = int(perception_cfg.get("lab_red_a_min", 150))
                lab_red_b_min = int(perception_cfg.get("lab_red_b_min", 140))
                lab_gray_a_dev = int(perception_cfg.get("lab_gray_a_dev", 15))
                lab_gray_b_dev = int(perception_cfg.get("lab_gray_b_dev", 15))
                heading_connect_close_px = int(heading_cfg.get("connect_close_px", 1))
                heading_merge_gap_px = int(heading_cfg.get("merge_gap_px", 1))
                heading_p1_margin_px = int(heading_cfg.get("p1_margin_px", 1))

            frame = camera.read()
            height, width = frame.shape[:2]

            pts_src, top_y, bottom_y = calculate_roi_points(width, height, roi_top, roi_bottom)
            frame_with_roi = apply_roi_overlay(frame, pts_src, width, height, top_y, bottom_y)

            warped, _ = warp_perspective(frame, pts_src, ipm_resolution)
            binary = detect_road_lines(
                warped,
                detect_value,
                lab_red_l_min,
                lab_red_a_min,
                lab_red_b_min,
                lab_gray_a_dev,
                lab_gray_b_dev,
            )

            error_norm, centroid_x = compute_lane_error(binary)
            slope_norm, heading_centers, heading_offset, target_mask = estimate_heading(
                binary,
                connect_close_px=heading_connect_close_px,
                merge_gap_px=heading_merge_gap_px,
                p1_margin_px=heading_p1_margin_px,
            )
            if heading_offset is not None:
                heading_prev = heading_smooth_alpha * heading_offset + (1 - heading_smooth_alpha) * heading_prev
                heading_used = heading_prev
            else:
                heading_used = None
            now = time.perf_counter()
            dt = now - last_time
            last_time = now

            if heading_used is None:
                direction = "LOST"
                steering_output = 0.0
            else:
                steering_output = pid.update(heading_used, dt)
                direction = "STRAIGHT"

            # 진행 상태 판별 (좌/우 회전/직진/길잃음) - heading 기반
            state = direction
            if heading_used is None:
                state = "LOST"
            else:
                turn_by_heading = slope_norm is not None and abs(slope_norm) > turn_slope_thresh
                turn_by_offset = abs(heading_used) > turn_offset_thresh
                if turn_by_heading or turn_by_offset:
                    if heading_used > 0:
                        state = "TURN_RIGHT"
                    else:
                        state = "TURN_LEFT"
                else:
                    state = "STRAIGHT"

            # 상태별 속도/조향 스케일 조정
            effective_speed = base_speed_slider
            effective_steer_scale = steer_scale_slider
            if state.startswith("TURN"):
                effective_speed = int(base_speed_slider * turn_speed_scale)
                effective_steer_scale = steer_scale_slider * turn_steer_scale

            controller.base_speed = effective_speed
            controller.steer_scale = effective_steer_scale

            applied_left_speed = 0
            applied_right_speed = 0
            if motors_enabled:
                if state == "LOST":
                    controller.stop()
                    if runtime_cfg.get("fail_safe_beep", True):
                        hardware.beep(0.05)
                else:
                    applied_left_speed, applied_right_speed, drive_dir = controller.drive(steering_output)
                    direction = state if state != "STRAIGHT" else drive_dir
            else:
                # 모터 일시정지 상태: 출력만 갱신, 실제 구동은 중단
                if motors_was_enabled:
                    controller.stop()
                direction = "PAUSE"
                state = "PAUSE"

            fps = fps_timer.lap()

            if runtime_cfg.get("print_debug", False) and motors_enabled:
                print(
                    f"heading_err={heading_used if heading_used is not None else 'None'} "
                    f"slope={slope_norm if slope_norm is not None else 'None'} "
                    f"state={state} "
                    f"steer={steering_output:.2f} "
                    f"speed_l={applied_left_speed} "
                    f"speed_r={applied_right_speed}"
                )

            if show_windows:
                debug_input = cv2.bitwise_not(target_mask) if target_mask is not None else binary
                debug_binary = visualize_binary_debug(
                    debug_input,
                    direction,
                    centroid_x,
                    steering_output,
                    fps,
                    heading=slope_norm,
                    heading_offset=heading_used,
                    turn_thresholds=(turn_slope_thresh, turn_offset_thresh),
                    centers=heading_centers,
                )
                cv2.imshow("phase1/frame", frame_with_roi)
                cv2.imshow("phase1/binary", debug_binary)

                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
                elif key == ord("s"):  # 's' 키: 모터 토글
                    motors_enabled = not motors_enabled
                    controller.stop()
                    if motors_enabled:
                        print("모터 재개")
                    else:
                        print("모터 일시정지 (영상/슬라이더 계속)")
                elif key == 32:  # 스페이스바: 일시정지
                    controller.stop()
                    print("일시정지. 아무 키나 누르면 재개...")
                    cv2.waitKey()  # 키 입력 대기
            else:
                time.sleep(runtime_cfg.get("headless_delay", 0.01))
            motors_was_enabled = motors_enabled

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
