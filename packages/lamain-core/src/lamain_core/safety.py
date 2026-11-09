# SPDX-License-Identifier: Apache-2.0
"""F6: V1 guardrails.

Layers: software joint clamping (clamp or strict), hardware limits (min/max
position registers, written by the controller), per-mode torque, step limiting,
temperature/voltage/load monitoring, watchdog, and forbidding crossing the
dead center of the crank joints.
"""
from __future__ import annotations

from dataclasses import dataclass

from . import units
from .calibration import HandCalibration
from .model import HandModel


class SafetyError(RuntimeError):
    pass


@dataclass
class JointLimit:
    name: str
    servo_id: int
    reference_ticks: int
    direction: int
    min_ticks: int
    max_ticks: int


class SafetyFilter:
    def __init__(self, calib: HandCalibration, model: HandModel, mode: str = "clamp"):
        if mode not in ("clamp", "strict"):
            raise ValueError("mode must be clamp|strict")
        self.mode = mode
        self.model = model
        # Demo/playback may relax the per-cycle step cap (bounds are still
        # always enforced). None = use the model value.
        self.max_step_override_deg: float | None = None
        self.limits: dict[str, JointLimit] = {
            name: JointLimit(
                name=name,
                servo_id=c.servo_id,
                reference_ticks=c.reference_ticks,
                direction=c.direction,
                min_ticks=c.min_ticks,
                max_ticks=c.max_ticks,
            )
            for name, c in calib.joints.items()
        }

    def torque_pct(self, mode: str) -> float:
        saf = self.model.safety
        table = {
            "calib": self.model.calibration.torque_pct,
            "free": saf.free_torque_pct,
            "default": saf.default_torque_pct,
            "grasp": saf.grasp_torque_pct,
        }
        if mode not in table:
            raise ValueError(f"unknown torque mode: {mode!r}")
        return table[mode]

    def max_step_ticks(self, name: str) -> int:
        if self.max_step_override_deg is not None:
            return units.deg_to_ticks(self.max_step_override_deg, self.model.servo)
        return units.deg_to_ticks(
            self.model.safety.max_step_deg_per_cycle, self.model.servo
        )

    def clamp_ticks(self, name: str, desired: int) -> int:
        lim = self.limits[name]
        if desired < lim.min_ticks or desired > lim.max_ticks:
            if self.mode == "strict":
                raise SafetyError(
                    f"{name}: command {desired} out of range "
                    f"[{lim.min_ticks}, {lim.max_ticks}]"
                )
            return max(lim.min_ticks, min(lim.max_ticks, desired))
        return desired

    def limit_step(self, name: str, desired: int, previous: int | None) -> int:
        if previous is None:
            return desired
        step = self.max_step_ticks(name)
        if desired > previous + step:
            return previous + step
        if desired < previous - step:
            return previous - step
        return desired

    def check_health(self, temperature: int, voltage: float, load: int) -> str:
        """Return 'ok' | 'warn' | 'stop'."""
        pre = self.model.preconditions
        if temperature >= pre.abort_temperature:
            return "stop"
        if not (pre.min_voltage <= voltage <= pre.max_voltage):
            return "stop"
        if temperature >= pre.warn_temperature:
            return "warn"
        return "ok"
