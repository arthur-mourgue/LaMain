# SPDX-License-Identifier: Apache-2.0
"""Servo access: common interface, real rustypot driver and FakeBus.

The rest of the library only talks to `ServoBus`, never directly to rustypot.
Register addresses are avoided: access goes through rustypot's register table,
by name. The servo family (SCS0009, STS3036, ...) comes from `ServoSpec`, so
the bus itself contains no family-specific constant.
"""
from __future__ import annotations

import random
import time
from dataclasses import dataclass
from typing import Protocol

from . import units
from .units import ServoSpec

TORQUE_ON = 1
TORQUE_OFF = 2

TORQUE_REG_FULL_SCALE = 1000  # register 16/48: 1000 = 100 %
STALL_LOAD = 1000.0           # load (present_load) when stalled at 100 % torque


class BusError(RuntimeError):
    pass


class ServoNotFound(BusError):
    pass


def _scalar(value):
    """rustypot returns a sequence (one element) even for a single servo."""
    if isinstance(value, (int, float, bool)):
        return value
    try:
        return value[0]
    except (TypeError, IndexError, KeyError):
        return value


def controller_class(servo: ServoSpec):
    """Resolve the rustypot controller class from the model's servo spec."""
    import rustypot

    name = servo.controller
    cls = getattr(rustypot, name, None)
    if cls is None or not name.endswith("PyController"):
        raise ValueError(
            f"unknown rustypot controller {name!r} for servo {servo.model!r}"
        )
    return cls


def make_controller(servo: ServoSpec, port: str, baudrate: int, timeout: float = 0.5):
    """Open a rustypot controller for the CLI bus tools."""
    return controller_class(servo)(
        serial_port=port, baudrate=baudrate, timeout=timeout
    )


def change_servo_id(
    port: str,
    servo: ServoSpec,
    old_id: int,
    new_id: int,
    baudrate: int = 1_000_000,
    timeout: float = 0.5,
) -> None:
    """Change one servo's ID, opening the EEPROM lock first.

    Uses rustypot's `Bus` API, which works for both the SCS and STS families.
    The caller must have exactly one servo on the bus.
    """
    from rustypot import Bus

    definition = controller_class(servo).definition()
    # Include the new id in the map so the EEPROM lock can be closed again
    # once the servo answers under its new id.
    motors = {old_id: definition, new_id: definition}
    with Bus(port, baudrate, timeout, motors) as bus:
        bus.change_id(definition, old_id, new_id)
        try:
            bus.write_register(new_id, "lock", 1)
        except Exception:
            # Some firmware applies the id only at the next power cycle.
            bus.write_register(old_id, "lock", 1)


class ServoBus(Protocol):
    """Minimal interface used by calibration, safety and controller."""

    simulated: bool

    def scan(self) -> dict[int, int]: ...
    def read_position(self, servo_id: int) -> int: ...
    def read_load(self, servo_id: int) -> int: ...
    def read_temperature(self, servo_id: int) -> int: ...
    def read_voltage(self, servo_id: int) -> float: ...
    def read_moving(self, servo_id: int) -> int: ...
    def write_goal(self, servo_id: int, ticks: int) -> None: ...
    def set_goal_speed(self, servo_id: int, speed: float) -> None: ...
    def set_torque_enable(self, servo_id: int, on: bool) -> None: ...
    def read_torque_enable(self, servo_id: int) -> int: ...
    def set_torque_limit_pct(self, servo_id: int, pct: float) -> None: ...
    def read_register(self, servo_id: int, name: str) -> int: ...
    def write_register(self, servo_id: int, name: str, value: int) -> None: ...
    def step(self) -> None: ...
    def close(self) -> None: ...


# --------------------------------------------------------------------------- #
# Real driver
# --------------------------------------------------------------------------- #
class RustypotBus:
    simulated = False

    def __init__(
        self,
        port: str,
        servo: ServoSpec,
        baudrate: int = 1_000_000,
        timeout: float = 0.2,
    ):
        self.servo = servo
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._c = controller_class(servo)(
            serial_port=port, baudrate=baudrate, timeout=timeout
        )

    def scan(self) -> dict[int, int]:
        return dict(self._c.scan())

    def _require(self, servo_id: int) -> None:
        if not self._c.ping(servo_id):
            raise ServoNotFound(f"servo {servo_id} not on the bus")

    def _read_reg(self, servo_id: int, name: str) -> int:
        try:
            return int(_scalar(self._c.read_register(servo_id, name, retries=2)))
        except Exception as exc:
            raise BusError(f"read {name} servo {servo_id}: {exc}") from exc

    def _write_reg(self, servo_id: int, name: str, value: int) -> None:
        try:
            self._c.write_register(servo_id, name, int(value), retries=2)
        except Exception as exc:
            raise BusError(f"write {name} servo {servo_id}: {exc}") from exc

    def read_position(self, servo_id: int) -> int:
        # Raw position register (0..resolution-1), by name so the family's word
        # order and encoding are handled by rustypot.
        return self._read_reg(servo_id, "present_position")

    def read_load(self, servo_id: int) -> int:
        return self._read_reg(servo_id, "present_load")

    def read_temperature(self, servo_id: int) -> int:
        return self._read_reg(servo_id, "present_temperature")

    def read_voltage(self, servo_id: int) -> float:
        # present_voltage register: raw value in 0.1 V on both families.
        return self._read_reg(servo_id, "present_voltage") / 10.0

    def read_moving(self, servo_id: int) -> int:
        return self._read_reg(servo_id, "moving")

    def write_goal(self, servo_id: int, ticks: int) -> None:
        self._write_reg(servo_id, "goal_position", int(ticks))

    def set_goal_speed(self, servo_id: int, speed: float) -> None:
        # On the STS3036 firmware, acceleration 0 leaves the servo motionless
        # (verified on the bench): write the model ramp before the speed.
        if self.servo.acceleration is not None:
            self._write_reg(servo_id, "acceleration", self.servo.acceleration)
        if self.servo.speed_unit == "steps_per_s":
            # STS goal_speed is a signed 16-bit value in steps/s.
            self._write_reg(servo_id, "goal_speed", int(round(speed)))
            return
        # SCS rustypot speed step: 0 = max speed, 1..6 = speed step.
        try:
            self._c.write_goal_speed(servo_id, float(speed))
        except Exception as exc:
            raise BusError(f"write goal_speed servo {servo_id}: {exc}") from exc

    def set_torque_enable(self, servo_id: int, on: bool) -> None:
        self._write_reg(
            servo_id,
            "torque_enable",
            self.servo.torque_on if on else self.servo.torque_off,
        )

    def read_torque_enable(self, servo_id: int) -> int:
        return self._read_reg(servo_id, "torque_enable")

    def set_torque_limit_pct(self, servo_id: int, pct: float) -> None:
        value = int(round(max(0.0, min(100.0, pct)) * TORQUE_REG_FULL_SCALE / 100.0))
        self._write_reg(servo_id, self.servo.torque_register, value)

    def read_register(self, servo_id: int, name: str) -> int:
        return self._read_reg(servo_id, name)

    def write_register(self, servo_id: int, name: str, value: int) -> None:
        self._write_reg(servo_id, name, int(value))

    def step(self) -> None:  # hardware does not need to be advanced
        return None

    def close(self) -> None:
        self._c.close()


# --------------------------------------------------------------------------- #
# FakeBus (tests, CI)
# --------------------------------------------------------------------------- #
@dataclass
class FakeServo:
    servo_id: int
    stop_min: int
    stop_max: int
    assembly_position: int = 511
    position: int = 511
    goal: int = 511
    speed_ticks: int = 24          # ticks travelled per step()
    torque_enabled: bool = True
    torque_limit_pct: float = 100.0
    fault: str | None = None       # "absent" | "overtemp"
    noise: int = 0
    friction_load: int = 20


class FakeBus:
    """Hand simulator: stops, dead center, load at the stop, noise, faults."""

    simulated = True

    def __init__(
        self,
        servos: list[FakeServo],
        seed: int = 0,
        servo: ServoSpec | None = None,
    ):
        self._servos = {s.servo_id: s for s in servos}
        self._rng = random.Random(seed)
        self.servo = servo

    # construction ---------------------------------------------------------- #
    @classmethod
    def standard_hand(cls, model, seed: int = 0) -> "FakeBus":
        servo = model.servo
        servos = []
        for name in model.calibration_order:
            j = model.joint(name)
            half = units.deg_to_ticks(j.nominal_travel_deg / 2.0, servo)
            asm = j.assembly_position
            stop_min = max(servo.tick_min, asm - half)
            stop_max = min(servo.tick_max, asm + half)
            servos.append(
                FakeServo(
                    servo_id=j.servo_id,
                    stop_min=stop_min,
                    stop_max=stop_max,
                    assembly_position=asm,
                    position=asm,
                    goal=asm,
                    speed_ticks=max(1, round(24 * servo.resolution / 1024)),
                )
            )
        return cls(servos, seed=seed, servo=servo)

    # protocol -------------------------------------------------------------- #
    def scan(self) -> dict[int, int]:
        return {
            s.servo_id: 7777
            for s in self._servos.values()
            if s.fault != "absent"
        }

    def _s(self, servo_id: int) -> FakeServo:
        s = self._servos.get(servo_id)
        if s is None or s.fault == "absent":
            raise ServoNotFound(f"servo {servo_id} not on the bus")
        return s

    def read_position(self, servo_id: int) -> int:
        return self._s(servo_id).position

    def read_load(self, servo_id: int) -> int:
        s = self._s(servo_id)
        blocked = s.goal > s.stop_max or s.goal < s.stop_min
        if not blocked or not s.torque_enabled:
            return s.friction_load
        return int(STALL_LOAD * s.torque_limit_pct / 100.0)

    def read_temperature(self, servo_id: int) -> int:
        s = self._s(servo_id)
        return 70 if s.fault == "overtemp" else 30

    def read_voltage(self, servo_id: int) -> float:
        self._s(servo_id)
        return 6.0

    def read_moving(self, servo_id: int) -> int:
        s = self._s(servo_id)
        return 1 if s.goal != s.position else 0

    def write_goal(self, servo_id: int, ticks: int) -> None:
        self._s(servo_id).goal = int(ticks)

    def set_goal_speed(self, servo_id: int, speed: float) -> None:
        self._s(servo_id)  # validate existence

    def set_torque_enable(self, servo_id: int, on: bool) -> None:
        self._s(servo_id).torque_enabled = bool(on)

    def read_torque_enable(self, servo_id: int) -> int:
        on = self._s(servo_id).torque_enabled
        if self.servo is not None:
            return self.servo.torque_on if on else self.servo.torque_off
        return TORQUE_ON if on else TORQUE_OFF

    def set_torque_limit_pct(self, servo_id: int, pct: float) -> None:
        self._s(servo_id).torque_limit_pct = max(0.0, min(100.0, pct))

    def read_register(self, servo_id: int, name: str) -> int:
        self._s(servo_id)
        if name == "present_temperature":
            return self.read_temperature(servo_id)
        if name == "present_voltage":
            return int(self.read_voltage(servo_id) * 10)
        if name in ("max_torque_limit", "torque_limit"):
            return int(self._s(servo_id).torque_limit_pct * TORQUE_REG_FULL_SCALE / 100)
        raise BusError(f"register {name!r} not simulated")

    def write_register(self, servo_id: int, name: str, value: int) -> None:
        self._s(servo_id)
        if name in ("max_torque_limit", "torque_limit"):
            self._s(servo_id).torque_limit_pct = value * 100.0 / TORQUE_REG_FULL_SCALE
        else:
            raise BusError(f"register {name!r} not simulated")

    def step(self) -> None:
        for s in self._servos.values():
            if s.fault == "absent" or not s.torque_enabled:
                continue
            if s.goal > s.position:
                s.position = min(s.goal, s.position + s.speed_ticks)
            elif s.goal < s.position:
                s.position = max(s.goal, s.position - s.speed_ticks)
            if s.noise:
                s.position += self._rng.randint(-s.noise, s.noise)
            s.position = max(s.stop_min, min(s.stop_max, s.position))

    def close(self) -> None:
        return None


# --------------------------------------------------------------------------- #
# Ramped move (shared by calibration, diagnostics and the CLI)
# --------------------------------------------------------------------------- #
def move_to(
    bus: ServoBus,
    servo_id: int,
    target: int,
    max_step: int,
    timeout_s: float,
    settle_s: float = 0.02,
) -> bool:
    """Step the goal towards `target`; return True when reached.

    The STS3036 ignores a direct goal jump larger than about 1000 ticks (the
    servo simply does not move). Every caller must therefore ramp the goal in
    `max_step` increments instead of writing the final position once.
    """
    deadline = time.monotonic() + timeout_s
    max_step = max(1, int(max_step))
    while time.monotonic() < deadline:
        current = bus.read_position(servo_id)
        if abs(current - target) <= 1:
            return True
        step = max(-max_step, min(max_step, target - current))
        bus.write_goal(servo_id, current + step)
        bus.step()
        if not bus.simulated:
            time.sleep(settle_s)
    return False
