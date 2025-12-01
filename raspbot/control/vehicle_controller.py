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
    ) -> None:
        self.hardware = hardware
        self.base_speed = base_speed
        self.speed_limit = speed_limit
        self.steer_scale = steer_scale
        self.deadband = deadband

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def drive(self, steering_command: float) -> Tuple[int, int, str]:
        steer = steering_command * self.steer_scale
        if abs(steer) < self.deadband:
            steer = 0.0

        left_speed = self.base_speed - steer
        right_speed = self.base_speed + steer

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
