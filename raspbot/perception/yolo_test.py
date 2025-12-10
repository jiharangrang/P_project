"""YOLOv8n 카메라 테스트."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

import cv2
from ultralytics import YOLO

from raspbot.hardware.camera import Camera, CameraConfig


def resolve_model_path(model_arg: Optional[str]) -> Path:
    if model_arg:
        model_path = Path(model_arg).expanduser().resolve()
    else:
        model_path = Path(__file__).resolve().parents[2] / "models" / "yolo" / "best.pt"

    if not model_path.exists():
        raise FileNotFoundError(f"YOLO 모델 파일을 찾을 수 없습니다: {model_path}")
    return model_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLOv8n 카메라 테스트")
    parser.add_argument("--model", type=str, help="모델(.pt) 경로. 기본: models/yolo/best.pt")
    parser.add_argument("--imgsz", type=int, default=320, help="입력 이미지 크기")
    parser.add_argument("--conf", type=float, default=0.5, help="신뢰도 임계값")
    parser.add_argument("--device", type=str, default=None, help="cuda, cpu 등 강제 디바이스 지정")
    parser.add_argument("--cam-index", type=int, default=0, help="카메라 인덱스")
    parser.add_argument("--width", type=int, default=320, help="카메라 가로 해상도")
    parser.add_argument("--height", type=int, default=240, help="카메라 세로 해상도")
    return parser.parse_args()


def main():
    args = parse_args()
    model_path = resolve_model_path(args.model)

    print(f"[INFO] YOLO 모델 로드: {model_path}")
    model = YOLO(str(model_path))
    if args.device:
        model.to(args.device)

    print("[INFO] 모델 클래스 목록:")
    for cls_id, name in model.names.items():
        print(f"  {cls_id}: {name}")

    cam_cfg = CameraConfig(
        index=args.cam_index, resolution=(args.width, args.height)
    )
    camera = Camera(cam_cfg)
    print(f"[INFO] 카메라 초기화 완료 (index={args.cam_index}, res={args.width}x{args.height})")
    print("[INFO] ESC 또는 q 키를 누르면 종료합니다.")

    try:
        while True:
            try:
                frame = camera.read()
            except RuntimeError as e:
                print(f"[WARN] 프레임을 읽지 못했습니다: {e}")
                continue

            results = model(frame, imgsz=args.imgsz, conf=args.conf, verbose=False)
            annotated = results[0].plot()

            cv2.imshow("YOLOv8n Test", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                print("[INFO] 종료 키 입력. 프로그램을 종료합니다.")
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()
        print("[INFO] 정리 완료.")


if __name__ == "__main__":
    main()
