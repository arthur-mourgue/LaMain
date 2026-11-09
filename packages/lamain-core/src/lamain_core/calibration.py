# SPDX-License-Identifier: Apache-2.0
"""F3/F4: exhaustive automatic calibration.

Sequence:
  - set every servo to 0 degrees (ASSEMBLY zero);
  - pause so the user can fit the servo horns (hand extended);
  - for each joint, one finger at a time: full-travel sweep in both directions,
    robust stop detection (stagnation + torque confirmation), compute the joint
    zero, return to the assembly zero.

Two zeros are stored per joint:
  - mount_ticks     : assembly zero (mount pose), used for resets;
  - reference_ticks : joint zero (API q = 0), like MiddlePos.

Joint types:
  - ``two_stop`` : a stop on each side (abduction, thumb base) -> reference =
    the side with the greatest reach (``reference_side: auto``);
  - ``crank``    : crank, extension = dead center (not a stop) -> reference =
    the midpoint of the two flexion stops, crossing forbidden.
"""
from __future__ import annotations

import csv
import statistics
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

from . import units
from .bus import ServoBus
from .model import HandModel, JointModel


class CalibrationAbort(RuntimeError):
    """Immediate stop: cut torque, report the error."""


@dataclass
class SweepSample:
    t: float
    servo_id: int
    goal: int
    position: int
    load: int
    temperature: int


@dataclass
class StopResult:
    ticks: int
    at_servo_end: bool
    repeats: list[int] = field(default_factory=list)

    @property
    def spread(self) -> int:
        return (max(self.repeats) - min(self.repeats)) if len(self.repeats) > 1 else 0


@dataclass
class JointCalibration:
    name: str
    servo_id: int
    joint_type: str
    mount_ticks: int
    reference_ticks: int
    direction: int
    stop_low_ticks: int
    stop_high_ticks: int
    min_ticks: int
    max_ticks: int
    home_ticks: int
    dead_center_ticks: int = 0
    repeatability_ticks: int = 0
    low_at_servo_end: bool = False
    high_at_servo_end: bool = False
    status: str = "ok"
    cause: str = ""
    measured_travel_deg: float = 0.0
    nominal_travel_deg: float = 0.0

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class CalibrationLog:
    rows: list[SweepSample] = field(default_factory=list)

    def add(self, s: SweepSample) -> None:
        self.rows.append(s)

    def write_csv(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["t", "servo_id", "goal", "position", "load", "temperature"])
            for s in self.rows:
                writer.writerow(
                    [f"{s.t:.4f}", s.servo_id, s.goal, s.position, s.load, s.temperature]
                )


# --------------------------------------------------------------------------- #
# low level
# --------------------------------------------------------------------------- #
def _settle(bus: ServoBus) -> None:
    bus.step()


def _sleep(bus: ServoBus, model: HandModel) -> None:
    if not bus.simulated:
        time.sleep(model.calibration.settle_s)


def _move_to(bus: ServoBus, model: HandModel, servo_id: int, target: int) -> bool:
    """Move to `target`; return True if reached (within 1 tick)."""
    target = max(model.servo.tick_min, min(model.servo.tick_max, target))
    # 600 * 20 ms = 12 s, enough for a full STS travel at the model speed.
    for _ in range(600):
        if abs(bus.read_position(servo_id) - target) <= 1:
            return True
        bus.write_goal(servo_id, target)
        _settle(bus)
        if not bus.simulated:
            time.sleep(0.02)
    return False


def _read(
    bus: ServoBus,
    model: HandModel,
    servo_id: int,
    goal: int,
    log: CalibrationLog,
) -> tuple[int, int]:
    _settle(bus)
    _sleep(bus, model)
    pos = bus.read_position(servo_id)
    load = bus.read_load(servo_id)
    temp = bus.read_temperature(servo_id)
    log.add(SweepSample(time.monotonic(), servo_id, goal, pos, load, temp))
    if temp >= model.preconditions.abort_temperature:
        raise CalibrationAbort(
            f"servo {servo_id} : surchauffe {temp} C >= "
            f"{model.preconditions.abort_temperature}"
        )
    return pos, load


def _scan_torque(model: HandModel, joint: JointModel) -> float:
    """Torque used while sweeping (same for every joint)."""
    return model.calibration.torque_pct


def _probe_torque(model: HandModel, joint: JointModel) -> float:
    """Torque used to push/confirm a stop (same for every joint)."""
    return model.calibration.probe_torque_pct


def _confirm_stop(
    bus: ServoBus,
    joint: JointModel,
    model: HandModel,
    direction: int,
    pos: int,
    log: CalibrationLog,
    trace=None,
) -> bool:
    """At a suspected stop, push at max torque for up to `probe_max_s` seconds
    to really confirm it (dwell), then release torque.

    Returns True if the position does not move despite the push (a real stop).
    """
    cfg = model.calibration
    sid = joint.servo_id
    scan = _scan_torque(model, joint)
    probe = _probe_torque(model, joint)
    target = max(
        model.servo.tick_min,
        min(model.servo.tick_max, pos + direction * cfg.overrun_ticks),
    )
    if trace is not None:
        trace(f"      contact at {pos}: push {probe:.0f}% for {cfg.probe_max_s:.1f}s...")
    bus.set_torque_limit_pct(sid, probe)
    moved = 0
    try:
        deadline = time.monotonic() + cfg.probe_max_s
        steps = 0
        while time.monotonic() < deadline:
            bus.write_goal(sid, target)
            p, _ = _read(bus, model, sid, target, log)
            moved = abs(p - pos)
            if moved > cfg.stable_epsilon_ticks:
                break
            steps += 1
            if bus.simulated and steps >= 3:
                break
    finally:
        bus.set_torque_limit_pct(sid, scan)
    if trace is not None:
        trace("      -> stop confirmed" if moved <= cfg.stable_epsilon_ticks
              else "      -> it moves (friction), continuing")
    return moved <= cfg.stable_epsilon_ticks


def _approach(
    bus: ServoBus,
    joint: JointModel,
    model: HandModel,
    direction: int,
    step: int,
    log: CalibrationLog,
    deadline: float | None = None,
    trace=None,
) -> StopResult:
    """Step in `direction` until a confirmed stop or the servo's internal end."""
    cfg = model.calibration
    sid = joint.servo_id
    servo = model.servo
    max_iters = (servo.tick_max - servo.tick_min) // max(1, step) + 200
    goal = bus.read_position(sid)
    # Net-progress stall detection: robust to position jitter. We track the
    # furthest position reached; if it stops improving for `stable_reads` reads
    # while the command keeps leading, the joint is stalled.
    best = goal
    no_progress = 0

    for i in range(max_iters):
        if deadline is not None and time.monotonic() > deadline:
            raise CalibrationAbort(
                f"servo {sid}: timeout ({cfg.joint_timeout_s:.0f}s) in "
                f"{direction:+d} (joint stuck or slow bus?)"
            )
        if trace is not None and i % max(1, cfg.progress_every) == 0:
            trace(f"    servo {sid} {direction:+d}: goal={goal} pos={bus.read_position(sid)}")

        goal = max(servo.tick_min, min(servo.tick_max, goal + direction * step))
        bus.write_goal(sid, goal)
        pos, load = _read(bus, model, sid, goal, log)

        if direction < 0:
            improved = pos < best - cfg.stable_epsilon_ticks
        else:
            improved = pos > best + cfg.stable_epsilon_ticks
        if improved:
            best = pos
            no_progress = 0
        else:
            no_progress += 1

        at_end = (
            direction < 0 and pos <= servo.tick_min + cfg.stable_epsilon_ticks
        ) or (
            direction > 0 and pos >= servo.tick_max - cfg.stable_epsilon_ticks
        )
        goal_limit = (direction < 0 and goal <= servo.tick_min) or (
            direction > 0 and goal >= servo.tick_max
        )
        overrun = (goal - pos) * direction
        stalled = no_progress >= cfg.stable_reads and (
            overrun >= cfg.overrun_ticks or goal_limit
        )

        if at_end:
            # servo internal end: still push at max torque so the load rises
            # (same behaviour as a mechanical stop) and is logged.
            _confirm_stop(bus, joint, model, direction, pos, log, trace)
            return StopResult(pos, True, [pos])
        if stalled:
            # At the command boundary we cannot push further: accept the stop.
            if goal_limit or _confirm_stop(bus, joint, model, direction, pos, log, trace):
                return StopResult(pos, False, [pos])
            best = pos  # friction slipped: restart the progress window
            no_progress = 0

    raise CalibrationAbort(
        f"servo {sid}: no stop found in {direction:+d} (travel exceeded)"
    )


def _search_stop(
    bus: ServoBus,
    joint: JointModel,
    model: HandModel,
    direction: int,
    log: CalibrationLog,
    deadline: float | None = None,
    trace=None,
) -> StopResult:
    """Exhaustive stop search: fast pass, back off, repeated slow re-approach,
    median.

    We start from the CURRENT POSITION (no forced zero): if the servo horn is
    mounted against a stop, we find it immediately and go look for the other one.
    """
    cfg = model.calibration
    sid = joint.servo_id

    approx = _approach(
        bus, joint, model, direction, cfg.fast_step_ticks, log, deadline, trace
    )

    backoff = units.deg_to_ticks(cfg.backoff_deg, model.servo)
    _move_to(bus, model, sid, approx.ticks - direction * backoff)

    repeats: list[int] = []
    at_end = approx.at_servo_end
    for _ in range(max(1, cfg.repeat)):
        res = _approach(
            bus, joint, model, direction, cfg.step_ticks, log, deadline, trace
        )
        repeats.append(res.ticks)
        at_end = at_end or res.at_servo_end
        _move_to(bus, model, sid, res.ticks - direction * backoff)

    return StopResult(int(statistics.median(repeats)), at_end, repeats)


# --------------------------------------------------------------------------- #
# zero and range computation
# --------------------------------------------------------------------------- #
def _reference_for(
    joint: JointModel,
    mount: int,
    stop_low: int,
    stop_high: int,
    reference_mode: str = "middle",
) -> tuple[int, int]:
    """Return (reference_ticks, direction).

    `reference_mode`:
      - "middle": q=0 = midpoint of the two stops (symmetric both ways);
      - "mount" : q=0 = assembly zero (assembly_position).
    """
    if reference_mode == "mount":
        reference = mount
        # if the assembly zero is outside the reachable travel (e.g. mounted
        # against a stop), fall back to the middle so q=0 stays reachable.
        if not (stop_low <= reference <= stop_high):
            reference = (stop_low + stop_high) // 2
    else:
        reference = (stop_low + stop_high) // 2
    direction = joint.branch_sign if joint.is_crank else 1
    return reference, direction


def _working_range(
    model: HandModel,
    joint: JointModel,
    reference: int,
    direction: int,
    stop_low: int,
    stop_high: int,
) -> tuple[int, int]:
    """Full measured travel (both stops), kept inside by `home_margin_deg`."""
    margin = units.deg_to_ticks(model.safety.home_margin_deg, model.servo)
    lo, hi = stop_low + margin, stop_high - margin
    lo = max(model.servo.tick_min, lo)
    hi = min(model.servo.tick_max, hi)
    if lo > hi:
        lo = hi = reference
    return lo, hi


def _verify(
    cal: JointCalibration,
    joint: JointModel,
    compare_nominal: bool,
    repeat_tol: int,
    end_margin: int,
    tick_max: int,
) -> None:
    if cal.measured_travel_deg < 20.0:
        cal.status = "failed"
        cal.cause = (
            f"travel {cal.measured_travel_deg:.1f} deg almost zero: "
            "joint stuck or stop not reached"
        )
        return
    # both stops at the servo ends = no mechanical stop
    if cal.stop_low_ticks <= end_margin and cal.stop_high_ticks >= tick_max - end_margin:
        cal.status = "failed"
        cal.cause = (
            "both stops are the servo's internal limit "
            "(detached linkage or no mechanical stop)"
        )
        return
    if cal.repeatability_ticks > repeat_tol:
        cal.status = "uncertain"
        cal.cause = (
            f"repeatability {cal.repeatability_ticks} ticks > {repeat_tol}: "
            "soft stop / friction, not a firm stop"
        )
        return
    # for two_stop only: stop stuck at the servo travel end
    if not joint.is_crank and (cal.low_at_servo_end or cal.high_at_servo_end):
        side = "low" if cal.low_at_servo_end else "high"
        cal.status = "uncertain"
        cal.cause = (
            f"{side} stop within {end_margin} ticks of the servo travel end: "
            "probably the servo limit, not a mechanical stop "
            "(reposition the horn or force reference_side: stop_low/stop_high)"
        )
        return
    if not compare_nominal or joint.nominal_travel_deg <= 0:
        cal.status = "ok"
        cal.cause = ""
        return
    tol = joint.range_tolerance_deg
    delta = cal.measured_travel_deg - joint.nominal_travel_deg
    if abs(delta) <= tol:
        cal.status = "ok"
        cal.cause = ""
    elif delta > 0:
        cal.status = "failed"
        cal.cause = (
            f"travel {cal.measured_travel_deg:.0f} deg > nominal "
            f"{joint.nominal_travel_deg:.0f} deg +{tol}: detached linkage or "
            "servo internal stop"
        )
    else:
        cal.status = "failed"
        cal.cause = (
            f"travel {cal.measured_travel_deg:.0f} deg < nominal "
            f"{joint.nominal_travel_deg:.0f} deg -{tol}: horn off by a spline "
            "(1 tooth=18 deg), collision or friction likely"
        )


# --------------------------------------------------------------------------- #
# preconditions and joint
# --------------------------------------------------------------------------- #
def preflight(bus: ServoBus, model: HandModel) -> list[str]:
    problems: list[str] = []
    try:
        found = bus.scan()
    except Exception as exc:  # pragma: no cover - hardware
        return [f"bus scan failed: {exc}"]

    expected = {j.servo_id for j in model.joints.values()}
    missing = sorted(expected - set(found))
    if missing:
        problems.append(f"missing servos: {missing}")

    if not bus.simulated and model.servo.model_number is not None:
        wrong = {
            sid: model_no
            for sid, model_no in found.items()
            if sid in expected and model_no != model.servo.model_number
        }
        if wrong:
            problems.append(
                f"unexpected servo model(s): {wrong} "
                f"(expected {model.servo.model_number})"
            )

    for sid in sorted(expected & set(found)):
        volt = bus.read_voltage(sid)
        if not (model.preconditions.min_voltage <= volt <= model.preconditions.max_voltage):
            problems.append(
                f"servo {sid}: voltage {volt:.1f} V outside "
                f"[{model.preconditions.min_voltage}, {model.preconditions.max_voltage}]"
            )
        temp = bus.read_temperature(sid)
        if temp >= model.preconditions.abort_temperature:
            problems.append(
                f"servo {sid}: {temp} C >= {model.preconditions.abort_temperature}"
            )
    return problems


def calibrate_joint(
    bus: ServoBus,
    joint: JointModel,
    model: HandModel,
    log: CalibrationLog,
    verify: bool = False,
    trace=None,
) -> JointCalibration:
    cfg = model.calibration
    sid = joint.servo_id
    mount = joint.assembly_position

    bus.set_torque_limit_pct(sid, _scan_torque(model, joint))
    bus.set_goal_speed(sid, model.safety.move_speed)
    # Hold the current pose before enabling torque: a stale goal would make the
    # joint jump at power-on.
    bus.write_goal(sid, bus.read_position(sid))
    bus.set_torque_enable(sid, True)

    deadline = time.monotonic() + cfg.joint_timeout_s
    if trace is not None:
        trace("    low stop (direction -1)...")
    low = _search_stop(bus, joint, model, -1, log, deadline, trace)
    if trace is not None:
        trace("    high stop (direction +1)...")
    high = _search_stop(bus, joint, model, +1, log, deadline, trace)
    stop_low, stop_high = sorted((low.ticks, high.ticks))
    end_margin = cfg.servo_end_margin_ticks
    low_at_end = stop_low <= model.servo.tick_min + end_margin
    high_at_end = stop_high >= model.servo.tick_max - end_margin

    reference, direction = _reference_for(
        joint, mount, stop_low, stop_high,
        model.calibration.reference_mode,
    )
    lo, hi = _working_range(model, joint, reference, direction, stop_low, stop_high)
    margin = units.deg_to_ticks(model.safety.home_margin_deg, model.servo)
    home = reference + direction * margin
    home = max(stop_low + margin, min(stop_high - margin, home))

    cal = JointCalibration(
        name=joint.name,
        servo_id=sid,
        joint_type=joint.joint_type,
        mount_ticks=int(mount),
        reference_ticks=int(reference),
        direction=int(direction),
        stop_low_ticks=int(stop_low),
        stop_high_ticks=int(stop_high),
        min_ticks=int(lo),
        max_ticks=int(hi),
        home_ticks=int(home),
        dead_center_ticks=int(reference) if joint.is_crank else 0,
        repeatability_ticks=max(low.spread, high.spread),
        low_at_servo_end=bool(low_at_end),
        high_at_servo_end=bool(high_at_end),
        measured_travel_deg=units.ticks_to_deg(stop_high - stop_low, model.servo),
        nominal_travel_deg=joint.nominal_travel_deg,
    )
    _verify(
        cal,
        joint,
        compare_nominal=verify,
        repeat_tol=model.calibration.repeat_tolerance_ticks,
        end_margin=cfg.servo_end_margin_ticks,
        tick_max=model.servo.tick_max,
    )
    _move_to(bus, model, sid, mount)  # return to the ASSEMBLY zero
    return cal


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
@dataclass
class HandCalibration:
    hand_serial: str
    model_version: str
    joints: dict[str, JointCalibration]
    servo_model: str = ""
    created_at: str = ""
    software_version: str = "0.1.0"
    log_path: str = ""

    def to_dict(self) -> dict:
        return {
            "hand_serial": self.hand_serial,
            "model_version": self.model_version,
            "servo_model": self.servo_model,
            "created_at": self.created_at,
            "software_version": self.software_version,
            "joints": {name: c.to_dict() for name, c in self.joints.items()},
        }

    @property
    def valid(self) -> bool:
        # "uncertain" is a warning, not a blocker; only "failed" blocks.
        return bool(self.joints) and all(
            c.status != "failed" for c in self.joints.values()
        )

    @property
    def warnings(self) -> list[str]:
        return [name for name, c in self.joints.items() if c.status == "uncertain"]


def _prepare_all(bus: ServoBus, model: HandModel) -> dict[str, int]:
    """Set EVERY servo to its ASSEMBLY zero (assembly_position)."""
    positions: dict[str, int] = {}
    for j in model.joints.values():
        bus.set_torque_limit_pct(j.servo_id, model.calibration.torque_pct)
        bus.set_goal_speed(j.servo_id, model.safety.move_speed)
        # Write the assembly goal before enabling torque, so a stale goal can
        # never move the joint during the reset.
        bus.write_goal(j.servo_id, j.assembly_position)
        bus.set_torque_enable(j.servo_id, True)
        _move_to(bus, model, j.servo_id, j.assembly_position)
        positions[j.name] = bus.read_position(j.servo_id)
    return positions


def calibrate_hand(
    bus: ServoBus,
    model: HandModel,
    hand_serial: str,
    logs_dir: Path | None = None,
    verify: bool = False,
    step_by_step: bool = True,
    progress=None,
    on_reset=None,
    mount_confirm=None,
    trace=None,
) -> HandCalibration:
    problems = preflight(bus, model)
    if problems:
        raise CalibrationAbort("; ".join(problems))

    log = CalibrationLog()
    results: dict[str, JointCalibration] = {}
    order = list(model.calibration_order)
    total = len(order)

    def _reset(label: str) -> None:
        positions = _prepare_all(bus, model)
        if on_reset is not None:
            on_reset(label, positions)

    try:
        _reset("depart")
        if mount_confirm is not None:
            mount_confirm()
        for idx, name in enumerate(order):
            joint = model.joint(name)
            if progress is not None:
                progress(name, joint.servo_id, idx + 1, total)
            if step_by_step:
                _reset(f"before {name}")
            results[name] = calibrate_joint(
                bus, joint, model, log, verify=verify, trace=trace
            )
            if step_by_step:
                _reset(f"after {name}")
    finally:
        for j in model.joints.values():
            try:
                bus.set_torque_enable(j.servo_id, False)
            except Exception:
                pass

    if logs_dir is not None:
        log.write_csv(Path(logs_dir) / "sweep.csv")

    from datetime import datetime, timezone

    return HandCalibration(
        hand_serial=hand_serial,
        model_version=model.model_version,
        servo_model=model.servo.model,
        joints=results,
        created_at=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        log_path=str(Path(logs_dir) / "sweep.csv") if logs_dir else "",
    )
