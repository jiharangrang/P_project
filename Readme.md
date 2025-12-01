# P Project (Raspbot V2)

Phase 1용 라인 기반 자율주행 파이프라인을 모듈화했습니다. `raspbot/` 아래에 하드웨어(HAL), 인지, 제어, 실행 루프가 분리되어 있으며 설정은 `configs/phase1_pid.yaml`에서 관리합니다.

## 실행
- 기본 설정으로 실행: `python3 scripts/run_phase1.py`
- 다른 설정 파일 사용: `python3 scripts/run_phase1.py --config configs/phase1_pid.yaml`
- 실기 없이 로직만 점검: `python3 scripts/run_phase1.py --mock-hw`
- 디스플레이 없이 실행: `python3 scripts/run_phase1.py --headless`
