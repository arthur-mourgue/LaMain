# SPDX-License-Identifier: Apache-2.0
"""Read `hand_model.yaml`: the design description, shipped with this package.

Every parameter (torques, steps, thresholds, margins, order) comes from this
file. No "magic" value should appear anywhere else in the code.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from . import paths
from .units import ServoSpec

REFERENCE_STRATEGIES = {"dead_center_midpoint", "hard_stop_min", "hard_stop_max"}
JOINT_TYPES = {"two_stop", "crank"}
REFERENCE_SIDES = {"auto", "stop_low", "stop_high", "mount"}

# Defaults for the two servo families this project has used. A model file can
# override every value under `bus.servo`, so a new servo needs no code change.
_SERVO_DEFAULTS: dict[str, dict[str, object]] = {
    "SCS0009": {
        "controller": "Scs0009PyController",
        "resolution": 1024,
        "range_deg": 300.0,
        "torque_register": "max_torque_limit",
        "torque_on": 1,
        "torque_off": 2,
        "speed_unit": "step",
        "model_number": 1284,
    },
    "STS3036": {
        "controller": "Sts3215PyController",
        "resolution": 4096,
        "range_deg": 360.0,
        "torque_register": "torque_limit",
        "torque_on": 1,
        "torque_off": 0,
        "speed_unit": "steps_per_s",
        "acceleration": 50,
        "model_number": 521,
    },
    "STS3215": {
        "controller": "Sts3215PyController",
        "resolution": 4096,
        "range_deg": 360.0,
        "torque_register": "torque_limit",
        "torque_on": 1,
        "torque_off": 0,
        "speed_unit": "steps_per_s",
        "acceleration": 50,
        "model_number": 777,
    },
}


@dataclass(frozen=True)
class JointModel:
    name: str
    servo_id: int
    assembly_position: int
    reference: str
    branch: str
    range_tolerance_deg: float
    nominal_travel_deg: float
    max_speed_dps: float
    joint_type: str = "two_stop"
    reference_side: str = "auto"
    calibration_torque_pct: float | None = None
    probe_torque_pct: float | None = None

    def __post_init__(self) -> None:
        if self.reference not in REFERENCE_STRATEGIES:
            raise ValueError(
                f"{self.name}: unknown reference {self.reference!r} "
                f"(expected: {sorted(REFERENCE_STRATEGIES)})"
            )
        if self.branch not in ("positive", "negative"):
            raise ValueError(f"{self.name}: branch must be positive|negative")
        if self.joint_type not in JOINT_TYPES:
            raise ValueError(f"{self.name}: type must be {sorted(JOINT_TYPES)}")
        if self.reference_side not in REFERENCE_SIDES:
            raise ValueError(
                f"{self.name}: reference_side must be {sorted(REFERENCE_SIDES)}"
            )

    @property
    def is_crank(self) -> bool:
        return self.joint_type == "crank"

    @property
    def branch_sign(self) -> int:
        return 1 if self.branch == "positive" else -1


@dataclass(frozen=True)
class BusConfig:
    port: str = "/dev/ttyACM0"
    baudrate: int = 1_000_000
    servo_model: str = "SCS0009"


def _servo_from_bus(bus_raw: dict) -> ServoSpec:
    """Build the servo spec from `bus.servo`, with family defaults.

    A model file that only says `servo_model: SCS0009` still loads (the archived
    SCS design); a new family is described entirely by `bus.servo`.
    """
    name = str(bus_raw.get("servo_model", "SCS0009"))
    raw = dict(_SERVO_DEFAULTS.get(name, {}))
    raw.update(bus_raw.get("servo") or {})
    raw.setdefault("model", name)
    for key in ("controller", "resolution", "range_deg"):
        if key not in raw:
            raise ValueError(
                f"bus.servo.{key} is required for servo_model {name!r}"
            )
    acceleration = raw.get("acceleration")
    model_number = raw.get("model_number")
    return ServoSpec(
        model=str(raw["model"]),
        controller=str(raw["controller"]),
        resolution=int(raw["resolution"]),
        range_deg=float(raw["range_deg"]),
        torque_register=str(raw.get("torque_register", "max_torque_limit")),
        torque_on=int(raw.get("torque_on", 1)),
        torque_off=int(raw.get("torque_off", 2)),
        speed_unit=str(raw.get("speed_unit", "step")),
        acceleration=None if acceleration is None else int(acceleration),
        model_number=None if model_number is None else int(model_number),
    )


@dataclass(frozen=True)
class CalibrationConfig:
    step_ticks: int = 1
    fast_step_ticks: int = 8
    stable_reads: int = 10
    stable_epsilon_ticks: int = 1
    overrun_ticks: int = 20
    load_threshold: int = 0
    torque_pct: float = 25.0
    probe_torque_pct: float = 70.0
    probe_max_s: float = 0.3
    backoff_deg: float = 7.0
    repeat: int = 2
    settle_s: float = 0.05
    servo_end_margin_ticks: int = 40
    repeat_tolerance_ticks: int = 2
    reference_mode: str = "middle"   # "middle" (midpoint of stops) | "mount" (assembly zero)
    joint_timeout_s: float = 60.0    # max time per joint before aborting
    progress_every: int = 30         # print a progress line every N reads


@dataclass(frozen=True)
class Preconditions:
    min_voltage: float = 4.8
    max_voltage: float = 7.4
    warn_temperature: int = 50
    abort_temperature: int = 55


@dataclass(frozen=True)
class SafetyConfig:
    home_margin_deg: float = 5.0
    free_torque_pct: float = 20.0
    default_torque_pct: float = 40.0
    grasp_torque_pct: float = 80.0
    watchdog_timeout_s: float = 1.0
    max_step_deg_per_cycle: float = 10.0
    move_speed: float = 3.0


@dataclass(frozen=True)
class HandModel:
    model_version: str
    bus: BusConfig
    servo: ServoSpec
    calibration_order: tuple[str, ...]
    calibration: CalibrationConfig
    preconditions: Preconditions
    safety: SafetyConfig
    joints: dict[str, JointModel] = field(default_factory=dict)

    def joint(self, name: str) -> JointModel:
        try:
            return self.joints[name]
        except KeyError:
            raise KeyError(f"unknown joint: {name!r}") from None

    def joint_by_servo_id(self, servo_id: int) -> JointModel:
        for j in self.joints.values():
            if j.servo_id == servo_id:
                return j
        raise KeyError(f"no joint for servo {servo_id}")


def _joint_from_dict(name: str, raw: dict) -> JointModel:
    return JointModel(
        name=name,
        servo_id=int(raw["servo_id"]),
        assembly_position=int(raw["assembly_position"]),
        reference=raw.get("reference", "hard_stop_min"),
        branch=raw.get("branch", "positive"),
        range_tolerance_deg=float(raw.get("range_tolerance_deg", 10.0)),
        nominal_travel_deg=float(raw.get("nominal_travel_deg", 0.0)),
        max_speed_dps=float(raw.get("max_speed_dps", 120.0)),
        joint_type=raw.get("type", "two_stop"),
        reference_side=raw.get("reference_side", "auto"),
        calibration_torque_pct=(
            None
            if raw.get("calibration_torque_pct") is None
            else float(raw["calibration_torque_pct"])
        ),
        probe_torque_pct=(
            None
            if raw.get("probe_torque_pct") is None
            else float(raw["probe_torque_pct"])
        ),
    )


def load_hand_model(path: str | Path) -> HandModel:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))

    bus_raw = data.get("bus", {})
    cal = data.get("calibration", {})
    pre = data.get("preconditions", {})
    saf = data.get("safety", {})
    joints_raw = data.get("joints", {})
    if not joints_raw:
        raise ValueError("hand_model.yaml: no joint defined")

    servo = _servo_from_bus(bus_raw)
    joints = {name: _joint_from_dict(name, raw) for name, raw in joints_raw.items()}

    order = tuple(data.get("calibration_order") or joints.keys())
    for name in order:
        if name not in joints:
            raise ValueError(f"calibration_order: unknown joint {name!r}")

    return HandModel(
        model_version=str(data.get("model_version", "unknown")),
        bus=BusConfig(
            port=str(bus_raw.get("port", "/dev/ttyACM0")),
            baudrate=int(bus_raw.get("baudrate", 1_000_000)),
            servo_model=servo.model,
        ),
        servo=servo,
        calibration_order=order,
        calibration=CalibrationConfig(
            step_ticks=int(cal.get("step_ticks", 1)),
            fast_step_ticks=int(cal.get("fast_step_ticks", cal.get("fast_travel_ticks", 8))),
            stable_reads=int(cal.get("stable_reads", 10)),
            stable_epsilon_ticks=int(cal.get("stable_epsilon_ticks", 1)),
            overrun_ticks=int(cal.get("overrun_ticks", 20)),
            load_threshold=int(cal.get("load_threshold", 0)),
            torque_pct=float(cal.get("torque_pct", 25.0)),
            probe_torque_pct=float(cal.get("probe_torque_pct", 70.0)),
            probe_max_s=float(cal.get("probe_max_s", 0.3)),
            backoff_deg=float(cal.get("backoff_deg", 7.0)),
            repeat=int(cal.get("repeat", 2)),
            settle_s=float(cal.get("settle_s", 0.05)),
            servo_end_margin_ticks=int(cal.get("servo_end_margin_ticks", 40)),
            repeat_tolerance_ticks=int(cal.get("repeat_tolerance_ticks", 2)),
            reference_mode=str(cal.get("reference_mode", "middle")),
            joint_timeout_s=float(cal.get("joint_timeout_s", 60.0)),
            progress_every=int(cal.get("progress_every", 30)),
        ),
        preconditions=Preconditions(
            min_voltage=float(pre.get("min_voltage", 4.8)),
            max_voltage=float(pre.get("max_voltage", 7.4)),
            warn_temperature=int(pre.get("warn_temperature", 50)),
            abort_temperature=int(pre.get("abort_temperature", 55)),
        ),
        safety=SafetyConfig(
            home_margin_deg=float(saf.get("home_margin_deg", 5.0)),
            free_torque_pct=float(saf.get("free_torque_pct", 20.0)),
            default_torque_pct=float(saf.get("default_torque_pct", 40.0)),
            grasp_torque_pct=float(saf.get("grasp_torque_pct", 80.0)),
            watchdog_timeout_s=float(saf.get("watchdog_timeout_s", 1.0)),
            max_step_deg_per_cycle=float(saf.get("max_step_deg_per_cycle", 10.0)),
            move_speed=float(saf.get("move_speed", 3.0)),
        ),
        joints=joints,
    )


def default_model_path() -> Path:
    """Default path, overridable with the LAMAIN_MODEL environment variable."""
    return paths.hand_model_path()
