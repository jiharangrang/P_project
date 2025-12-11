# P Project (Raspbot V2)

자율주행 Phase 1(주행 가능 도로 전체를 라인 추종하는 확장판) 파이프라인. 하드웨어 제어, 인지, 제어, 실행 루프를 모듈화했고 설정은 YAML 한 곳에 모아 관리합니다.

## 리포 구조
- `configs/phase1_pid.yaml` 설정 파일(ROI, Lab/HSV 임계, PID/턴/헤딩 파라미터, 런타임 옵션)
- `scripts/run_phase1.py` 실행 엔트리(항상 `raspbot.runtime.phase1_baseline`을 로드)
- `raspbot/`
  - `hardware/` 카메라/모터/서보 HAL (`RaspbotHardware`, `Camera`)
  - `control/` PID/차량 제어 (`PIDController`, `VehicleController`)
  - `perception/` 라인 검출(`lane_detection*.py`), IPM/ROI(`preprocessing.py`), 디버그 표시(`visualization.py`)
  - `runtime/` 실행 루프(`phase1_baseline*.py`), 상태 출력, 키 입력 처리
  - `utils/` FPS 측정, 설정 로더

## 실행하기
```bash
python3 scripts/run_phase1.py                # 기본 설정
python3 scripts/run_phase1.py --mode lab     # Lab 모드 명시
python3 scripts/run_phase1.py --mode hsv     # HSV 모드 명시
python3 scripts/run_phase1.py --config configs/phase1_pid.yaml  # 다른 설정 파일 사용
python3 scripts/run_phase1.py --mock-hw      # 모터/서보 호출을 콘솔 로그로 대체
python3 scripts/run_phase1.py --headless     # OpenCV 윈도우 없이 실행
```

## 라인 검출 모드 선택
- 설정: `perception.mode: lab | hsv` (default: lab)
- CLI: `python3 scripts/run_phase1.py --mode lab` 또는 `--mode hsv` (설정값을 덮어씀)
- 파일 교체 없이 모드에 따라 트랙바/파라미터도 자동으로 분기됩니다.

## 설정 파일(주요 항목)
- `perception`  
  - `mode`: `lab | hsv`  
  - 공통: `roi_top`, `roi_bottom`, `ipm_resolution`  
  - `hsv.detect_value`: 회색 밝기 임계  
  - `lab.detect_value`: L 밝기 임계  
  - `lab.red_l_min/red_a_min/red_b_min`, `lab.gray_a_dev/gray_b_dev`: Lab 임계값(L/a/b 최소 혹은 a/b 편차)
- `control`  
  - `base_speed`, `speed_limit`, `steer_limit`, `steer_scale`, `steer_deadband`  
  - PID: `kp`, `ki`, `kd`  
  - `turn`: `slope_thresh`, `offset_thresh`, `speed_scale`, `steer_scale`  
  - `heading`: `smooth_alpha`, `connect_close_px`, `merge_gap_px`, `p1_margin_px` (`inner_min_speed`는 Lab/HSV 변형에 미적용)
- `runtime`  
  - `show_windows`, `print_debug`, `fail_safe_beep`, `fps_window`, `headless_delay`, `enable_sliders`

## 런타임 제어(키)
- `ESC` / `q`: 종료
- `s`: 모터 시작/일시정지 토글(초기 상태: 정지)
- `SPACE`: 일시정지 후 아무 키나 눌러 재개

## 슬라이더(Windows 활성 시)
- ROI/밝기 임계: `roi_top`, `roi_bottom`, `detect_value`
- Lab 모드 전용: `lab_red_l_min`, `lab_red_a_min`, `lab_red_b_min`, `lab_gray_a_dev`, `lab_gray_b_dev`
- 조향/속도: `pid_kp_x100`, `pid_ki_x100`, `pid_kd_x100`, `base_speed`, `steer_scale_x100`
- 턴 파라미터: `turn_slope_thr_x100`, `turn_offset_thr_x100`, `turn_speed_scale_x100`, `turn_steer_scale_x100`
- 헤딩 파라미터: `heading_smooth_x100`, `heading_connect_close_px`, `heading_merge_gap_px`, `heading_p1_margin_px`
- 카메라/서보: `brightness`, `contrast`, `saturation`, `exposure`, `gain`, `servo1_yaw`, `servo2_pitch`

## 주요 처리 흐름
1) 프레임 입력 → ROI 계산/오버레이 (`calculate_roi_points`, `apply_roi_overlay`)  
2) IPM 변환 (`warp_perspective`)  
3) 이진화: HSV 모드 또는 Lab 모드 `detect_road_lines`  
4) 중심 오차 계산 (`compute_lane_error`)  
5) 헤딩 추정 (`estimate_heading`, `target_mask`로 도로 영역 연결/병합)  
6) PID로 조향 계산 → 상태별 속도/조향 스케일 적용 → `VehicleController.drive`  
7) 디버그 시각화(`visualize_binary_debug`): `target_mask` 반전 오버레이, 헤딩/상태/FPS 표시

## 디버그/로그
- 모터 활성 상태에서만 터미널 로그 출력: `heading_err`, `slope`, `state`, `steer`, `speed_l/r`
- OpenCV 창: `phase1/frame`(ROI 오버레이), `phase1/binary`(선택 도로 영역 강조)
- 모터 정지 시에는 출력/구동이 멈추며, `target_mask`를 반전한 디버그 바이너리가 계속 표시됩니다.

## 개발 팁
- 모드를 바꿀 때는 원하는 변형 파일을 `lane_detection.py`, `phase1_baseline.py`로 교체한 뒤 실행합니다.
- Lab 임계 조정은 `detect_value`(L 밝기)와 `lab_red_*`/`lab_gray_*` 슬라이더를 함께 조절해 그림자·반사에 맞추세요.
- HSV 모드에서 빨강 범위를 바꾸려면 `lane_detection_old.py` 내 `lower/upper_red` 값을 직접 수정해야 합니다.
