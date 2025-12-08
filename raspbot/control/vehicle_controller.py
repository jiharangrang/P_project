"""차량 구동 제어기."""

from __future__ import annotations

from typing import Tuple

from raspbot.hardware.raspbot import RaspbotHardware


class VehicleController:
    def __init__(
        self,
        hardware: RaspbotHardware,
        base_speed: int,
        speed_limit: int,
        steer_scale: float,
        deadband: float = 0.0,
        inner_min_speed: int = 0,
    ) -> None:
        self.hardware = hardware
        self.base_speed = base_speed
        self.speed_limit = speed_limit
        self.steer_scale = steer_scale
        self.deadband = deadband
        self.inner_min_speed = max(0, inner_min_speed)

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def drive(self, steering_command: float) -> Tuple[int, int, str]:
        steer = steering_command * self.steer_scale
        if abs(steer) < self.deadband:
            steer = 0.0

        # steer > 0: 오른쪽으로 돌기 위해 좌측 바퀴를 더 빠르게, 우측 바퀴를 더 느리게
        left_speed = self.base_speed + steer
        right_speed = self.base_speed - steer

        # 턴 시 안쪽 바퀴 최소 전진 속도 유지
        if steer > 1e-6:
            right_speed = max(right_speed, self.inner_min_speed)
        elif steer < -1e-6:
            left_speed = max(left_speed, self.inner_min_speed)

        left_speed = self._clamp(left_speed, -self.speed_limit, self.speed_limit)
        right_speed = self._clamp(right_speed, -self.speed_limit, self.speed_limit)

        self.hardware.drive(int(left_speed), int(right_speed))

        direction = "UP"
        if steer < -1e-3:
            direction = "LEFT"
        elif steer > 1e-3:
            direction = "RIGHT"

        return int(left_speed), int(right_speed), direction

    def stop(self) -> None:
        self.hardware.stop()
