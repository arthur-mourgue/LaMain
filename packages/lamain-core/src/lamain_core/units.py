# SPDX-License-Identifier: Apache-2.0
"""Servo specifications and unit conversions.

The servo model is data, not a constant: `ServoSpec` describes its resolution,
mechanical range and the register quirks that differ between families.

- SCS0009: 1024 ticks over 300 deg (0.293 deg/tick).
- STS3036: 4096 ticks over 360 deg (0.088 deg/tick).

Low-level logic works in raw ticks; radians only appear at the `HandController`
API edge. Every conversion goes through this module.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

SPEED_UNITS = {"step", "steps_per_s"}
TORQUE_REGISTERS = {"max_torque_limit", "torque_limit"}


@dataclass(frozen=True)
class ServoSpec:
    """Everything the code needs to know about one servo family."""

    model: str
    controller: str
    resolution: int
    range_deg: float
    torque_register: str = "max_torque_limit"
    torque_on: int = 1
    torque_off: int = 2
    speed_unit: str = "step"
    model_number: int | None = None

    def __post_init__(self) -> None:
        if self.resolution <= 1:
            raise ValueError("servo resolution must be > 1")
        if self.range_deg <= 0:
            raise ValueError("servo range_deg must be > 0")
        if self.torque_register not in TORQUE_REGISTERS:
            raise ValueError(
                f"torque_register must be one of {sorted(TORQUE_REGISTERS)}"
            )
        if self.speed_unit not in SPEED_UNITS:
            raise ValueError(f"speed_unit must be one of {sorted(SPEED_UNITS)}")
        if self.torque_on == self.torque_off:
            raise ValueError("torque_on and torque_off must differ")

    @property
    def tick_min(self) -> int:
        return 0

    @property
    def tick_max(self) -> int:
        return self.resolution - 1

    @property
    def mid_tick(self) -> int:
        """Electrical midpoint: the pose used to mount the horns."""
        return self.resolution // 2

    @property
    def ticks_per_deg(self) -> float:
        return self.resolution / self.range_deg

    @property
    def deg_per_tick(self) -> float:
        return self.range_deg / self.resolution

    @property
    def ticks_per_rad(self) -> float:
        return self.resolution / math.radians(self.range_deg)

    @property
    def rad_per_tick(self) -> float:
        return math.radians(self.range_deg) / self.resolution


def rad_to_ticks(rad: float, servo: ServoSpec) -> int:
    return int(round(rad * servo.ticks_per_rad))


def ticks_to_rad(ticks: float, servo: ServoSpec) -> float:
    return ticks * servo.rad_per_tick


def deg_to_ticks(deg: float, servo: ServoSpec) -> int:
    return int(round(deg * servo.ticks_per_deg))


def ticks_to_deg(ticks: float, servo: ServoSpec) -> float:
    return ticks * servo.deg_per_tick
