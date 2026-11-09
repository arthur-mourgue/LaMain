# SPDX-License-Identifier: Apache-2.0
"""F5: HandController, the single entry point to the servos.

Client code (teleoperation, AI) only uses joint names and radians (or normalized
coordinates [-1, 1]). It never sees a servo ID or a raw value. Every command
goes through the F6 safety filter.
"""
from __future__ import annotations

import math
from pathlib import Path

from . import units
from .bus import RustypotBus, ServoBus
from .calibration import HandCalibration
from .model import HandModel, default_model_path, load_hand_model
from .safety import SafetyFilter


def load_calibration(path: str | Path) -> HandCalibration:
    import json

    data = json.loads(Path(path).read_text(encoding="utf-8"))
    joints = {
        name: _joint_calibration_from_dict(name, raw)
        for name, raw in data["joints"].items()
    }
    return HandCalibration(
        hand_serial=data.get("hand_serial", "unknown"),
        model_version=data.get("model_version", "unknown"),
        servo_model=data.get("servo_model", ""),
        joints=joints,
        created_at=data.get("created_at", ""),
        software_version=data.get("software_version", "0.1.0"),
    )


def _joint_calibration_from_dict(name, raw):
    from .calibration import JointCalibration

    return JointCalibration(
        name=name,
        servo_id=int(raw["servo_id"]),
        joint_type=str(raw.get("joint_type", "two_stop")),
        mount_ticks=int(raw.get("mount_ticks", raw.get("reference_ticks", 511))),
        reference_ticks=int(raw["reference_ticks"]),
        direction=int(raw["direction"]),
        stop_low_ticks=int(raw["stop_low_ticks"]),
        stop_high_ticks=int(raw["stop_high_ticks"]),
        min_ticks=int(raw["min_ticks"]),
        max_ticks=int(raw["max_ticks"]),
        home_ticks=int(raw["home_ticks"]),
        dead_center_ticks=int(raw.get("dead_center_ticks", 0)),
        repeatability_ticks=int(raw.get("repeatability_ticks", 0)),
        low_at_servo_end=bool(raw.get("low_at_servo_end", False)),
        high_at_servo_end=bool(raw.get("high_at_servo_end", False)),
        status=str(raw.get("status", "ok")),
        cause=str(raw.get("cause", "")),
        measured_travel_deg=float(raw.get("measured_travel_deg", 0.0)),
        nominal_travel_deg=float(raw.get("nominal_travel_deg", 0.0)),
    )


class HandController:
    def __init__(
        self,
        bus: ServoBus,
        model: HandModel,
        calibration: HandCalibration,
        safety_mode: str = "clamp",
        require_valid: bool = True,
    ):
        if require_valid and not calibration.valid:
            raise RuntimeError(
                "invalid calibration: refusing to start. "
                "Run `lamain calibrate` (or fix the failed joints)."
            )
        if calibration.model_version != model.model_version:
            raise RuntimeError(
                f"calibration was made for model {calibration.model_version!r}, "
                f"not {model.model_version!r}: run `lamain calibrate`."
            )
        if calibration.servo_model and calibration.servo_model != model.servo.model:
            raise RuntimeError(
                f"calibration was made for servo {calibration.servo_model!r}, "
                f"not {model.servo.model!r}: run `lamain calibrate`."
            )
        self.bus = bus
        self.model = model
        self.calibration = calibration
        self.safety = SafetyFilter(calibration, model, mode=safety_mode)
        self._last_cmd: dict[str, int] = {}

    # lifecycle ------------------------------------------------------------- #
    @classmethod
    def from_files(
        cls,
        calibration_path: str | Path,
        model_path: str | Path | None = None,
        port: str | None = None,
        safety_mode: str = "clamp",
    ) -> "HandController":
        model = load_hand_model(model_path or default_model_path())
        calib = load_calibration(calibration_path)
        bus = RustypotBus(
            port or model.bus.port, model.servo, model.bus.baudrate
        )
        return cls(bus, model, calib, safety_mode=safety_mode)

    def connect(self) -> None:
        for j in self.model.joints.values():
            self.bus.set_torque_limit_pct(
                j.servo_id, self.model.safety.default_torque_pct
            )
            self.bus.set_goal_speed(j.servo_id, self.model.safety.move_speed)

    def disconnect(self) -> None:
        self.disable_torque()
        self.bus.close()

    def enable_torque(self, mode: str = "default") -> None:
        pct = self.safety.torque_pct(mode)
        for j in self.model.joints.values():
            self.bus.set_torque_limit_pct(j.servo_id, pct)
            self.bus.set_goal_speed(j.servo_id, self.model.safety.move_speed)
            # Hold the current pose before enabling torque: a stale goal (0
            # after a power cycle) would make the joint jump.
            self.bus.write_goal(j.servo_id, self.bus.read_position(j.servo_id))
            self.bus.set_torque_enable(j.servo_id, True)

    def disable_torque(self) -> None:
        for j in self.model.joints.values():
            try:
                self.bus.set_torque_enable(j.servo_id, False)
            except Exception:
                pass

    def set_max_step_deg(self, deg: float | None) -> None:
        """Relax (or restore) the per-cycle step cap, e.g. for fast playback.
        Joint bounds are still always enforced."""
        self.safety.max_step_override_deg = deg

    @property
    def joint_names(self) -> list[str]:
        return [j.name for j in self.model.joints.values()]

    # conversions ----------------------------------------------------------- #
    def _rad_to_ticks(self, name: str, rad: float) -> int:
        cal = self.calibration.joints[name]
        return cal.reference_ticks + cal.direction * units.rad_to_ticks(
            rad, self.model.servo
        )

    def _ticks_to_rad(self, name: str, ticks: int) -> float:
        cal = self.calibration.joints[name]
        return cal.direction * units.ticks_to_rad(
            ticks - cal.reference_ticks, self.model.servo
        )

    # joint API ------------------------------------------------------------- #
    def get_joint_positions(self) -> dict[str, float]:
        out: dict[str, float] = {}
        for j in self.model.joints.values():
            ticks = self.bus.read_position(j.servo_id)
            out[j.name] = self._ticks_to_rad(j.name, ticks)
        return out

    def set_joint_positions(self, positions: dict[str, float]) -> dict[str, float]:
        applied: dict[str, float] = {}
        for name, rad in positions.items():
            j = self.model.joint(name)
            ticks = self._rad_to_ticks(name, rad)
            ticks = self.safety.clamp_ticks(name, ticks)
            ticks = self.safety.limit_step(name, ticks, self._last_cmd.get(name))
            self.bus.write_goal(j.servo_id, ticks)
            self._last_cmd[name] = ticks
            applied[name] = self._ticks_to_rad(name, ticks)
        return applied

    # absolute tick API (calibration-independent playback on the same hand) --- #
    def get_joint_ticks(self) -> dict[str, int]:
        return {
            j.name: self.bus.read_position(j.servo_id)
            for j in self.model.joints.values()
        }

    def set_joint_ticks(self, values: dict[str, int]) -> dict[str, int]:
        applied: dict[str, int] = {}
        for name, ticks in values.items():
            j = self.model.joint(name)
            t = self.safety.clamp_ticks(name, int(ticks))
            t = self.safety.limit_step(name, t, self._last_cmd.get(name))
            self.bus.write_goal(j.servo_id, t)
            self._last_cmd[name] = t
            applied[name] = t
        return applied

    def mount_ticks(self) -> dict[str, int]:
        """The initial/mount zero in absolute ticks.

        Single source of truth: `assembly_position` from `hand_model.yaml`, so
        calibrate / inspect / assemble / studio all reset to the same values.
        """
        return {
            name: self.model.joint(name).assembly_position
            for name in self.calibration.joints
        }

    # normalized variants [-1, 1] (on the MEASURED range, never the nominal) #
    def _spans_ticks(self, name: str) -> tuple[int, int]:
        """(positive, negative) travel in ticks from the reference, i.e. how
        many ticks `q=+1` / `q=-1` cover."""
        cal = self.calibration.joints[name]
        ref = cal.reference_ticks
        if cal.direction > 0:
            return cal.max_ticks - ref, ref - cal.min_ticks
        return ref - cal.min_ticks, cal.max_ticks - ref

    def q_span_deg(self, name: str, sign: int) -> float:
        pos, neg = self._spans_ticks(name)
        return units.ticks_to_deg(pos if sign > 0 else neg, self.model.servo)

    def q_to_deg(self, name: str, q: float) -> float:
        """Joint angle in degrees for a normalized command q in [-1, 1]."""
        pos, neg = self._spans_ticks(name)
        span = units.ticks_to_rad(pos if q >= 0 else neg, self.model.servo)
        return math.degrees(q * span)

    def get_normalized(self) -> dict[str, float]:
        out: dict[str, float] = {}
        for name, rad in self.get_joint_positions().items():
            pos, neg = self._spans_ticks(name)
            span = units.ticks_to_rad(pos if rad >= 0 else neg, self.model.servo)
            out[name] = rad / span if span else 0.0
        return out

    def set_normalized(self, values: dict[str, float]) -> dict[str, float]:
        targets: dict[str, float] = {}
        for name, q in values.items():
            q = max(-1.0, min(1.0, q))
            pos, neg = self._spans_ticks(name)
            span = units.ticks_to_rad(pos if q >= 0 else neg, self.model.servo)
            targets[name] = q * span
        return self.set_joint_positions(targets)

    # hardware limits (EEPROM) ---------------------------------------------- #
    def write_hardware_limits(self) -> None:
        """Write min/max position to EEPROM (a fallback if the software crashes)."""
        if self.bus.simulated:
            return
        for name, cal in self.calibration.joints.items():
            j = self.model.joint(name)
            # EEPROM writes need the torque released and the lock open.
            self.bus.set_torque_enable(j.servo_id, False)
            self.bus.write_register(j.servo_id, "lock", 0)
            self.bus.write_register(j.servo_id, "min_position_limit", cal.min_ticks)
            self.bus.write_register(j.servo_id, "max_position_limit", cal.max_ticks)
            self.bus.write_register(j.servo_id, "lock", 1)
