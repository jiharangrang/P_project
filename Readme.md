# P Project (Raspbot V2)

Phase 1 라인 기반 자율주행 파이프라인을 모듈화한 프로젝트입니다. `raspbot/` 아래에 하드웨어(HAL), 인지, 제어, 실행 루프가 분리되어 있으며 설정은 YAML로 관리됩니다.

## 디렉터리
- `scripts/run_phase1.py` : 엔트리 포인트. CLI 인자 파싱 후 `raspbot/runtime/phase1_baseline.py` 실행.
- `raspbot/runtime/phase1_baseline.py` : 메인 주행 루프(ROI+IPM 기반 라인 추종).
- `raspbot/hardware/` : `Raspbot_Lib` 래퍼(`RaspbotHardware`)와 `Camera` 캡처/설정.
- `raspbot/perception/` : ROI 계산, IPM(warp), HSV/그레이 임계 기반 라인 검출, heading 분석, 디버그 시각화.
- `raspbot/control/` : PID 계산(`PIDController`)과 속도/조향 변환(`VehicleController`).
- `configs/phase1_pid.yaml` : 하드웨어·인지·제어·런타임 파라미터 기본값.
- `0_autoplot___test.py` : 초기 Raspbot 예제/실습 스크립트(메인 루프와 별개, 참고용).

## 실행 방법
- 기본 실행: `python3 scripts/run_phase1.py`
- 다른 설정 사용: `python3 scripts/run_phase1.py --config configs/phase1_pid.yaml`
- 실기 없이 모킹: `python3 scripts/run_phase1.py --mock-hw`
- 디스플레이 없이: `python3 scripts/run_phase1.py --headless`

| 옵션 | 기본값 | 설명 |
| --- | --- | --- |
| `--config` | `configs/phase1_pid.yaml` | YAML 설정 파일 경로 |
| `--mock-hw` | `False` | 실제 `Raspbot_Lib` 대신 콘솔 로그만 출력하는 `MockRaspbot` 사용 |
| `--headless` | `False` | OpenCV 윈도우 비활성 |

## 파이프라인 흐름 (`raspbot/runtime/phase1_baseline.py`)
1) 설정 로드 후 컴포넌트 생성: `RaspbotHardware`, `Camera`, `PIDController`, `VehicleController`. 슬라이더 옵션이 켜져 있으면 트랙바 UI를 띄워 실시간 파라미터 조정.  
2) 프레임 수집: `Camera.read()` → ROI 사각형 계산(`calculate_roi_points`)과 오버레이(`apply_roi_overlay`).  
3) IPM: ROI를 버드아이뷰 해상도(`ipm_resolution`)로 투영(`warp_perspective`).  
4) 라인 이진화: HSV로 빨간선 + 회색 밝기 임계(`detect_value`, 추가 어두움 마스킹) → 모폴로지 클린업.  
5) 에러/방향 추정:  
   - `compute_lane_error`: 도로(검정) 무게중심 x 좌표 계산(센트로이드).  
   - `estimate_heading`: 여러 높이 밴드의 중심 이동량으로 진행 기울기 및 상단 오프셋(-1~1) 계산, 상충 시 하단 우선 가중치 조정.  
   - Heading EMA(`heading_smooth_alpha`)로 노이즈 완화.  
6) 상태 판정: heading 기울기(`slope_norm`)와 오프셋이 턴 임계(`turn.slope_thresh`, `turn.offset_thresh`)를 넘으면 `TURN_LEFT/RIGHT`; 검출 실패 시 `LOST`, 그 외 `STRAIGHT`.  
7) 속도/조향 스케일: 상태가 TURN이면 `turn.speed_scale`, `turn.steer_scale`로 감속·조향 배율 적용.  
8) PID 제어: heading 오프셋을 PID 입력으로 `steer_limit` 범위 출력 → `VehicleController.drive`가 좌/우 속도로 변환하며 `steer_scale`·`deadband` 반영.  
9) 안전/피드백: `LOST` 시 정지 및 필요 시 Beep, FPS 계산(`FpsTimer`), 디버그 출력/시각화(`visualize_binary_debug`).  
10) 입력 처리: `ESC/q` 종료, `s` 모터 토글, `Space` 일시정지. 마지막에 모터 정지·서보 중립·LED 오프 정리.

## 설정 파일 (`configs/phase1_pid.yaml`)
### hardware
| 키 | 기본 | 설명 |
| --- | --- | --- |
| `use_led`, `use_beep` | `true` | LED/부저 사용 여부 |
| `led_on_start`, `beep_on_start` | `true` | 초기 시 LED 점등, 부저 테스트 |
| `led_mode` | `2` | `Ctrl_WQ2812_ALL` 모드 값 |
| `servo_defaults` | `[87, 29]` | 시작 시 서보(yaw, pitch) 위치 |
| `servo_neutral` | `[90, 25]` | 종료 시 서보 중립 위치 |
| `camera.*` | `index=0`, `resolution=[320,240]`, 밝기/대비/채도/노출/이득 | 캡처 및 트랙바 연동 설정 |

### perception
| 키 | 기본 | 설명 |
| --- | --- | --- |
| `roi_top`, `roi_bottom` | `756`, `946` (0~1000 스케일) | ROI 상·하단 위치 |
| `detect_value` | `170` | HSV/그레이 임계값 기준 |
| `ipm_resolution` | `[320, 240]` | IPM 후 해상도 |

### control
| 키 | 기본 | 설명 |
| --- | --- | --- |
| `base_speed`, `speed_limit` | `60`, `120` | 기본 속도 및 모터 포화 한계 |
| `steer_limit` | `80` | PID 출력 제한 |
| `steer_scale` / `steer_deadband` | `1.0` / `0.0` | 조향 배율 / 허용 데드밴드 |
| `kp`, `ki`, `kd` | `45.0`, `0.0`, `8.0` | PID 게인 |
| `turn.slope_thresh`, `turn.offset_thresh` | `0.01`, `0.10` | 턴 판정 임계 |
| `turn.speed_scale`, `turn.steer_scale` | `0.5`, `2.50` | 턴 시 감속/조향 배율 |
| `heading.smooth_alpha` | `0.20` | heading EMA 가중치 |

### runtime
| 키 | 기본 | 설명 |
| --- | --- | --- |
| `show_windows` | `true` | 디스플레이 출력 활성화 |
| `print_debug` | `true` | 콘솔 디버그 로그 |
| `fail_safe_beep` | `true` | 라인 상실 시 부저 알림 |
| `fps_window` | `15` | FPS 이동 평균 윈도우 |
| `headless_delay` | `0.02` | headless 모드 루프 슬립 |

## 실시간 조정 (트랙바)
- ROI/임계: `roi_top`, `roi_bottom`, `detect_value`.
- PID/속도: `pid_kp_x100`, `pid_ki_x100`, `pid_kd_x100`, `base_speed`, `steer_scale_x100`.
- 턴 판정/배율: `turn_slope_thr_x100`, `turn_offset_thr_x100`, `turn_speed_scale_x100`, `turn_steer_scale_x100`.
- Heading: `heading_smooth_x100`.
- 카메라: `brightness`, `contrast`, `saturation`, `exposure`, `gain`.
- 서보: `servo1_yaw`, `servo2_pitch` (pitch는 `SERVO_PITCH_LIMIT=110` 내 제한).

## 주요 컴포넌트 요약
- `raspbot/hardware/raspbot.py` : `Raspbot_Lib` 초기화, LED/부저/서보/모터 제어 및 정리, 모킹 가능.
- `raspbot/hardware/camera.py` : `cv2.VideoCapture` 래퍼, 해상도 설정 및 실시간 속성 반영.
- `raspbot/perception/preprocessing.py` : ROI 좌표 계산, ROI 오버레이, IPM 변환.
- `raspbot/perception/lane_detection.py` : HSV+그레이 기반 라인 이진화, 센트로이드 계산 및 heading 추정.
- `raspbot/perception/visualization.py` : 이진 디버그 뷰에 방향/조향/FPS/turn 임계/센터 포인트 표시.
- `raspbot/control/pid.py` : 적분/미분 포함 PID, 출력 클램프.
- `raspbot/control/vehicle_controller.py` : PID 결과를 좌/우 속도로 변환 후 하드웨어에 전달, 방향 상태 리턴.
- `raspbot/utils/config_loader.py`, `raspbot/utils/timing.py` : YAML 로드 및 FPS 계산.

## 운영 메모
- `lib/raspbot` 디바이스 라이브러리가 환경에 필요하며, `--mock-hw`로 의존성 없이 로직 테스트가 가능합니다.
- 라인 미검출(`LOST`) 시 즉시 정지하고 옵션에 따라 부저 알림을 수행합니다.
- 전/후진 로직은 포함하지 않으며, 좌우 조향은 모터 속도 차이로만 표현합니다.
