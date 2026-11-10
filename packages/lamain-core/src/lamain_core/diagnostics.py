# SPDX-License-Identifier: Apache-2.0
"""Diagnostic sweep of a joint.

Moves one joint slowly across its whole servo travel and records position,
load, temperature and voltage. It reports where it actually stops, whether each
stop looks like a real mechanical stop or the servo's internal end, and the
servo's EEPROM position limits. Works without a calibration (uses the model's
assembly position).
"""
from __future__ import annotations

import csv
import time
from dataclasses import dataclass, field
from pathlib import Path

from . import units
from .bus import ServoBus, move_to
from .model import HandModel, JointModel
from .units import ServoSpec


def _default_endpoint_margin(servo: ServoSpec) -> int:
    """20 ticks at SCS resolution (about 5.9 deg), scaled for the servo."""
    return max(2, round(20 * servo.resolution / 1024))


@dataclass
class SweepPoint:
    t: float
    goal: int
    position: int
    load: int
    temperature: int
    voltage: float


@dataclass
class JointDiagnosis:
    name: str
    servo_id: int
    mount_ticks: int
    stop_low: int
    stop_high: int
    load_max: int
    load_min: int
    temp_max: int
    voltage_min: float
    servo: ServoSpec
    servo_min_limit: int | None = None
    servo_max_limit: int | None = None
    endpoint_margin: int = 20
    points: list[SweepPoint] = field(default_factory=list)

    @property
    def travel_deg(self) -> float:
        return units.ticks_to_deg(
            self.stop_high - self.stop_low, self.servo
        )

    @property
    def low_is_servo_end(self) -> bool:
        return self.stop_low <= self.endpoint_margin

    @property
    def high_is_servo_end(self) -> bool:
        return self.stop_high >= self.servo.tick_max - self.endpoint_margin

    def _matches(self, stop: int, limit: int | None) -> bool:
        return limit is not None and abs(stop - limit) <= self.endpoint_margin

    @property
    def low_is_eeprom_limit(self) -> bool:
        return self._matches(self.stop_low, self.servo_min_limit)

    @property
    def high_is_eeprom_limit(self) -> bool:
        return self._matches(self.stop_high, self.servo_max_limit)

    @property
    def verdict(self) -> str:
        if self.low_is_servo_end and self.high_is_servo_end:
            return "both stops at the servo's internal ends (no distinct mechanical stop)"
        if self.low_is_servo_end or self.high_is_servo_end:
            side = "low" if self.low_is_servo_end else "high"
            return f"{side} stop at the servo's internal end (likely not mechanical)"
        return "two stops inside the servo travel (look like real mechanical stops)"

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "servo_id": self.servo_id,
            "mount_ticks": self.mount_ticks,
            "stop_low": self.stop_low,
            "stop_high": self.stop_high,
            "travel_deg": round(self.travel_deg, 2),
            "load_min": self.load_min,
            "load_max": self.load_max,
            "temp_max": self.temp_max,
            "voltage_min": round(self.voltage_min, 2),
            "servo_min_limit": self.servo_min_limit,
            "servo_max_limit": self.servo_max_limit,
            "low_is_servo_end": self.low_is_servo_end,
            "high_is_servo_end": self.high_is_servo_end,
            "verdict": self.verdict,
        }


def _move_to(
    bus: ServoBus, servo: ServoSpec, servo_id: int, target: int, settle_s: float
) -> None:
    target = max(servo.tick_min, min(servo.tick_max, target))
    move_to(
        bus,
        servo_id,
        target,
        max_step=max(50, servo.resolution // 12),
        timeout_s=8.0,
        settle_s=settle_s,
        tolerance=units.settle_tolerance(servo),
    )


def _read_servo_limits(bus: ServoBus, servo_id: int) -> tuple[int | None, int | None]:
    if bus.simulated:
        return None, None
    try:
        lo = bus.read_register(servo_id, "min_position_limit")
        hi = bus.read_register(servo_id, "max_position_limit")
        return int(lo), int(hi)
    except Exception:
        return None, None


def sweep_joint(
    bus: ServoBus,
    model: HandModel,
    joint: JointModel,
    step_ticks: int = 4,
    torque_pct: float | None = None,
    settle_s: float | None = None,
    endpoint_margin: int | None = None,
) -> JointDiagnosis:
    sid = joint.servo_id
    servo = model.servo
    if torque_pct is None:
        # push at probe torque so the joint really reaches its limits
        torque_pct = model.calibration.probe_torque_pct
    if settle_s is None:
        settle_s = model.calibration.settle_s
    if endpoint_margin is None:
        endpoint_margin = _default_endpoint_margin(servo)

    bus.set_torque_limit_pct(sid, torque_pct)
    bus.set_goal_speed(sid, model.safety.move_speed)
    bus.write_goal(sid, bus.read_position(sid))
    bus.set_torque_enable(sid, True)

    mount = joint.assembly_position
    _move_to(bus, servo, sid, mount, settle_s)

    points: list[SweepPoint] = []

    def record(goal: int) -> None:
        bus.step()
        if not bus.simulated:
            time.sleep(settle_s)
        points.append(
            SweepPoint(
                t=time.monotonic(),
                goal=goal,
                position=bus.read_position(sid),
                load=bus.read_load(sid),
                temperature=bus.read_temperature(sid),
                voltage=bus.read_voltage(sid),
            )
        )

    # sweep down to the servo's low end, then up to its high end
    goal = bus.read_position(sid)
    while goal > servo.tick_min:
        goal = max(servo.tick_min, goal - step_ticks)
        bus.write_goal(sid, goal)
        record(goal)
    while goal < servo.tick_max:
        goal = min(servo.tick_max, goal + step_ticks)
        bus.write_goal(sid, goal)
        record(goal)

    _move_to(bus, servo, sid, mount, settle_s)
    bus.set_torque_enable(sid, False)

    positions = [p.position for p in points] or [mount]
    loads = [p.load for p in points] or [0]
    temps = [p.temperature for p in points] or [0]
    volts = [p.voltage for p in points] or [6.0]
    servo_lo, servo_hi = _read_servo_limits(bus, sid)

    return JointDiagnosis(
        name=joint.name,
        servo_id=sid,
        mount_ticks=mount,
        stop_low=min(positions),
        stop_high=max(positions),
        load_max=max(loads),
        load_min=min(loads),
        temp_max=max(temps),
        voltage_min=min(volts),
        servo=servo,
        servo_min_limit=servo_lo,
        servo_max_limit=servo_hi,
        endpoint_margin=endpoint_margin,
        points=points,
    )


def write_csv(diag: JointDiagnosis, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t", "goal", "position", "load", "temperature", "voltage"])
        for p in diag.points:
            writer.writerow(
                [f"{p.t:.4f}", p.goal, p.position, p.load, p.temperature, f"{p.voltage:.2f}"]
            )


def plot(diag: JointDiagnosis, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    goals = [p.goal for p in diag.points]
    positions = [p.position for p in diag.points]
    loads = [p.load for p in diag.points]

    fig, ax1 = plt.subplots(figsize=(9, 5))
    ax1.plot(goals, positions, "b-", label="position")
    ax1.plot(goals, goals, "k--", alpha=0.3, label="command")
    ax1.set_xlabel("goal (ticks)")
    ax1.set_ylabel("position (ticks)", color="b")
    ax2 = ax1.twinx()
    ax2.plot(goals, loads, "r-", alpha=0.6, label="load")
    ax2.set_ylabel("load", color="r")
    ax1.axvline(diag.stop_low, color="g", ls=":", label=f"stop_low={diag.stop_low}")
    ax1.axvline(diag.stop_high, color="m", ls=":", label=f"stop_high={diag.stop_high}")
    fig.suptitle(
        f"{diag.name} (servo {diag.servo_id})  travel={diag.travel_deg:.1f} deg\n"
        f"{diag.verdict}"
    )
    ax1.legend(loc="upper left", fontsize=8)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
