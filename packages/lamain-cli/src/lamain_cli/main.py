# SPDX-License-Identifier: Apache-2.0
"""CLI `lamain` for La Main.

Subcommands:
  lamain bus scan | set-id OLD NEW | dump ID
  lamain assemble [--release]
  lamain calibrate [--verify] [--repeat N] [--tag NAME] [--simulate]
  lamain studio [--calibration FILE] [--step DEG]
  lamain demo play NAME [--loops N] [--speed X]
  lamain gesture list | show NAME | rename OLD NEW | delete NAME
  lamain episode list | show NAME | rename OLD NEW | delete NAME
  lamain inspect [JOINT] [--plot]
  lamain collect-collisions

Real commands always end with torque off (including on Ctrl+C). Use
`--simulate` to run the logic on a FakeBus.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from datetime import datetime
from pathlib import Path

from lamain_core import units
from lamain_core.bus import (
    FakeBus,
    RustypotBus,
    _scalar,
    change_servo_id,
    make_controller,
)
from lamain_core.calibration import CalibrationAbort, calibrate_hand
from lamain_core.controller import HandController, load_calibration
from lamain_core.demos import (
    EpisodeStore,
    GestureStore,
    PlayOptions,
    Player,
    PlayerStopped,
    Step,
)
from lamain_core.diagnostics import plot as diag_plot
from lamain_core.diagnostics import sweep_joint
from lamain_core.diagnostics import write_csv as diag_write_csv
from lamain_core.model import default_model_path, load_hand_model
from lamain_core import paths
from lamain_core.jog import DEFAULT_KEYMAP
from lamain_cli.keys import KeyReader
from lamain_cli.studio import Studio

REPO_ROOT = paths.repo_root()
DEFAULT_LOGS_DIR = paths.logs_dir()
DEFAULT_GESTURE_DIR = paths.gesture_dir()
DEFAULT_EPISODE_DIR = paths.episode_dir()
DEFAULT_SERIAL = paths.DEFAULT_SERIAL


def _bus(args, model):
    if getattr(args, "simulate", False):
        return FakeBus.standard_hand(model)
    port = getattr(args, "port", None) or model.bus.port
    return RustypotBus(port, model.servo, model.bus.baudrate)


def _move_timeout(model, minimum: float = 3.0) -> float:
    """Enough time for a full travel at the model speed (STS is 4096 ticks)."""
    if model.servo.speed_unit == "steps_per_s" and model.safety.move_speed > 0:
        return max(minimum, 1.2 * model.servo.resolution / model.safety.move_speed)
    return minimum


def _settle_tolerance(model) -> int:
    """2 ticks at SCS resolution, scaled to the servo."""
    return max(2, round(2 * model.servo.resolution / 1024))


def _model(args):
    return load_hand_model(getattr(args, "model", None) or default_model_path())


def _confirm(message: str, assume_yes: bool) -> bool:
    if assume_yes:
        return True
    try:
        answer = input(f"{message} [o/N] ").strip().lower()
    except EOFError:
        return False
    return answer in ("o", "oui", "y", "yes")


def _new_run_dir(kind: str, label: str | None = None, base=None) -> Path:
    """Create logs/<kind>/<timestamp>[_<label>][_N]/ for one run, so outputs stay
    segregated. A number is appended if the same name already exists."""
    root = Path(base or DEFAULT_LOGS_DIR)
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    stem = f"{timestamp}_{label}" if label else timestamp
    directory = root / kind / stem
    n = 2
    while directory.exists():
        directory = root / kind / f"{stem}_{n}"
        n += 1
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def _home_all(bus, model, timeout: float | None = None) -> None:
    """Bring every joint back to its initial mount zero before any operation."""
    tol = _settle_tolerance(model)
    for j in model.joints.values():
        bus.set_torque_limit_pct(j.servo_id, model.safety.default_torque_pct)
        bus.set_goal_speed(j.servo_id, model.safety.move_speed)
        # Goal first, torque second: never let a stale goal move the hand.
        bus.write_goal(j.servo_id, j.assembly_position)
        bus.set_torque_enable(j.servo_id, True)
    elapsed = 0.0
    deadline = timeout if timeout is not None else _move_timeout(model)
    while elapsed < deadline:
        bus.step()
        pos = {j.servo_id: bus.read_position(j.servo_id) for j in model.joints.values()}
        if all(
            abs(pos[j.servo_id] - j.assembly_position) <= tol
            for j in model.joints.values()
        ):
            return
        if not bus.simulated:
            time.sleep(0.1)
        elapsed += 0.1


# --------------------------------------------------------------------------- #
# F1 - bus
# --------------------------------------------------------------------------- #
def cmd_bus(args) -> int:
    model = _model(args)
    servo = model.servo
    port = args.port or model.bus.port
    c = make_controller(servo, port, model.bus.baudrate)
    try:
        if args.action == "scan":
            found = c.scan()
            if not found:
                print("No servo found. Check power, cable and jumper.")
                return 1
            for sid in sorted(found):
                volt = _scalar(c.read_register(sid, "present_voltage")) / 10.0
                temp = _scalar(c.read_register(sid, "present_temperature"))
                print(f"  ID {sid:>3}  model {found[sid]:>5}  {volt:.1f} V  {temp} C")
            return 0

        if args.action == "dump":
            for reg in c.registers():
                try:
                    value = _scalar(c.read_register(args.servo_id, reg.name))
                except Exception:
                    continue
                print(f"  {reg.name:28} = {value}")
            return 0

        # set-id
        old, new = args.old, args.new
        found = c.scan()
        if len(found) > 1:
            print(
                "Refused: several servos on the bus. "
                "Connect ONLY one servo to change an ID."
            )
            return 1
        if old not in found:
            print(f"Servo {old} not on the bus.")
            return 1
        # The Bus API needs exclusive access to the port.
        c.close()
        c = None
        change_servo_id(port, servo, old, new, model.bus.baudrate)
        print(f"ID {old} -> {new} OK.")
        c2 = make_controller(servo, port, model.bus.baudrate)
        try:
            for sid in sorted(c2.scan()):
                print(f"  present: {sid}")
        finally:
            c2.close()
        return 0
    finally:
        if c is not None:
            c.close()
    return 0


# --------------------------------------------------------------------------- #
# F2 - assemble
# --------------------------------------------------------------------------- #
def cmd_assemble(args) -> int:
    model = _model(args)
    bus = _bus(args, model)
    try:
        found = bus.scan()
        expected = {j.servo_id for j in model.joints.values()}
        missing = sorted(expected - set(found))
        if missing:
            print(f"Missing IDs: {missing}. Check assembly/power.")
            return 1
        if not bus.simulated and model.servo.model_number is not None:
            wrong = {
                sid: model_no
                for sid, model_no in found.items()
                if sid in expected and model_no != model.servo.model_number
            }
            if wrong:
                print(
                    f"Wrong servo model(s): {wrong}; "
                    f"expected {model.servo.model_number} ({model.servo.model})."
                )
                return 1
        print(f"Detected servos: {sorted(found)}")

        print("Positions BEFORE:")
        for name in model.calibration_order:
            j = model.joint(name)
            print(f"  ID {j.servo_id} ({name}): {bus.read_position(j.servo_id)} ticks")

        print("\nMoving to the assembly pose:")
        for name in model.calibration_order:
            j = model.joint(name)
            bus.set_torque_limit_pct(j.servo_id, model.safety.default_torque_pct)
            bus.set_goal_speed(j.servo_id, model.safety.move_speed)
            # Goal first, torque second: safe even if the servo's previous goal
            # was left at 0.
            bus.write_goal(j.servo_id, j.assembly_position)
            bus.set_torque_enable(j.servo_id, True)
            hint = (
                "horn up (full extension / dead center)"
                if j.reference.startswith("dead_center")
                else "finger in a neutral, unblocked position"
            )
            print(f"  ID {j.servo_id} ({name}) -> {j.assembly_position} ticks: {hint}")

        # wait for the servos to reach the target (or not)
        tol = _settle_tolerance(model)
        elapsed = 0.0
        deadline = _move_timeout(model)
        reached = {}
        while elapsed < deadline:
            reached = {
                j.servo_id: bus.read_position(j.servo_id) for j in model.joints.values()
            }
            if all(
                abs(reached[j.servo_id] - j.assembly_position) <= tol
                for j in model.joints.values()
            ):
                break
            if not bus.simulated:
                time.sleep(0.1)
            elapsed += 0.1

        print("\nPositions AFTER:")
        for name in model.calibration_order:
            j = model.joint(name)
            pos = reached.get(j.servo_id, bus.read_position(j.servo_id))
            delta = pos - j.assembly_position
            flag = "OK" if abs(delta) <= tol else f"DELTA {delta:+d} ticks (stuck?)"
            print(f"  ID {j.servo_id} ({name}): {pos} ticks  [{flag}]")

        if not _confirm(
            "Fit the servo horns in this pose, then press Enter.", args.yes
        ):
            print("Cancelled.")
            return 1

        if args.release:
            for j in model.joints.values():
                bus.set_torque_enable(j.servo_id, False)
            print("Torque off.")
        else:
            print("Torque on (horns stay in place). Ctrl-C to release.")
            while True:
                time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopped, torque off.")
    except Exception as exc:
        print(f"\nBUS/HARDWARE error: {exc}")
    finally:
        for j in model.joints.values():
            try:
                bus.set_torque_enable(j.servo_id, False)
            except Exception:
                pass
        bus.close()
    return 0


# --------------------------------------------------------------------------- #
# F3/F4 - calibrate
# --------------------------------------------------------------------------- #
def _print_hand(hand) -> None:
    print(f"\nHand {hand.hand_serial} ({hand.model_version}):")
    for name, c in hand.joints.items():
        print(
            f"  {name:16} {c.status:9} mount={c.mount_ticks:4} ref={c.reference_ticks:4} "
            f"stops=[{c.stop_low_ticks},{c.stop_high_ticks}] "
            f"travel={c.measured_travel_deg:5.1f} deg (nom {c.nominal_travel_deg:.0f}) "
            f"rep={c.repeatability_ticks}"
        )
        if c.cause:
            print(f"      -> {c.cause}")


def _run_calibration(
    args, model, bus, serial, mount: bool = False, logs_dir=None
) -> "object":
    if not _confirm(
        "The hand must be FREE (nothing between the fingers). Continue?", args.yes
    ):
        raise CalibrationAbort("cancelled by the user")

    def progress(name, servo_id, index, total):
        print(f"\n[{index}/{total}] calibrating {name} (ID {servo_id}) ...")

    def on_reset(label, positions):
        detail = "  ".join(f"{name}={ticks}" for name, ticks in positions.items())
        print(f"  zero ({label}): {detail}")

    def mount_confirm():
        print(
            "\nServos at 0 deg. Fit the servo horns for an EXTENDED hand "
            "(horns up), then validate."
        )
        if not args.yes:
            try:
                input("Press Enter to start calibration... ")
            except EOFError:
                pass

    run_dir = logs_dir or (Path(args.logs_dir) if args.logs_dir else DEFAULT_LOGS_DIR)
    return calibrate_hand(
        bus,
        model,
        hand_serial=serial,
        logs_dir=run_dir,
        verify=args.verify,
        step_by_step=not args.batch,
        progress=progress,
        on_reset=on_reset,
        mount_confirm=mount_confirm if mount else None,
        trace=print,
    )


def _next_calibration_path(calib_dir, serial: str, tag: str | None = None) -> Path:
    """Never overwrite an existing calibration: increment the suffix."""
    d = Path(calib_dir)
    d.mkdir(parents=True, exist_ok=True)
    if tag:
        return d / f"{serial}_{tag}.json"
    base = d / f"{serial}.json"
    if not base.exists():
        return base
    n = 2
    while True:
        candidate = d / f"{serial}_{n}.json"
        if not candidate.exists():
            return candidate
        n += 1


def cmd_calibrate(args) -> int:
    model = _model(args)
    serial = args.serial or DEFAULT_SERIAL
    bus = _bus(args, model)
    run_dir = _new_run_dir("calibration", serial, args.logs_dir)
    try:
        hand = _run_calibration(
            args, model, bus, serial, mount=True, logs_dir=run_dir
        )
        _print_hand(hand)
        if hand.warnings:
            print(
                "\nWARNING (to check): "
                + ", ".join(hand.warnings)
                + " -> doubtful stop (see reason above)."
            )

        if args.repeat and args.repeat > 1:
            print(f"\nRepeatability (--repeat {args.repeat}):")
            for _ in range(args.repeat - 1):
                again = _run_calibration(args, model, bus, serial, logs_dir=run_dir)
                worst = 0
                for name, c in hand.joints.items():
                    d = abs(c.reference_ticks - again.joints[name].reference_ticks)
                    worst = max(worst, d)
                print(f"  max reference spread = {worst} ticks (threshold 2)")

        out = _next_calibration_path(
            args.calib_dir or paths.calibration_dir(serial), serial, args.tag
        )
        out.write_text(json.dumps(hand.to_dict(), indent=2), encoding="utf-8")
        print(f"\nCalibration written: {out}")
        print("(previous calibrations are kept)")
        print(f"Sweep log: {run_dir / 'sweep.csv'}")
        return 0 if hand.valid else 2
    except CalibrationAbort as exc:
        print(f"\nABORT: {exc}")
        return 1
    except KeyboardInterrupt:
        print("\nCtrl-C: torque off.")
        return 130
    except Exception as exc:
        print(f"\nBUS/HARDWARE error: {exc}")
        print("Check the 6 V power and the USB cable, then retry.")
        return 1
    finally:
        try:
            for j in model.joints.values():
                bus.set_torque_enable(j.servo_id, False)
        except Exception:
            pass
        bus.close()


# --------------------------------------------------------------------------- #
# F7 - collect-collisions
# --------------------------------------------------------------------------- #
def cmd_collect(args) -> int:
    model = _model(args)
    bus = _bus(args, model)
    if args.out:
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
    else:
        out = _new_run_dir("collect", args.serial, args.logs_dir) / "poses.csv"
    try:
        for j in model.joints.values():
            bus.set_torque_enable(j.servo_id, False)
        print("Torque off. Place the hand near a collision, then press Enter.")
        print("Labels: contact | limit | free. 'q' to quit.")
        with out.open("a", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            if fh.tell() == 0:
                writer.writerow(["joints_rad", "label", "serial"])
            while True:
                try:
                    line = input("Enter to log (or a label, or q) > ").strip()
                except EOFError:
                    break
                if line.lower() == "q":
                    break
                label = line or "limit"
                pos = []
                for j in model.joints.values():
                    ticks = bus.read_position(j.servo_id)
                    pos.append(round(units.ticks_to_rad(ticks, model.servo), 6))
                writer.writerow([json.dumps(pos), label, args.serial])
                fh.flush()
                print(f"  logged ({label}): {pos}")
        print(f"Log: {out}")
        return 0
    finally:
        bus.close()


# --------------------------------------------------------------------------- #
# Studio / demo
# --------------------------------------------------------------------------- #
def _resolve_calibration(args) -> Path | None:
    if getattr(args, "calibration", None):
        return Path(args.calibration)
    if getattr(args, "calib_dir", None):
        return paths.latest_in_dir(args.calib_dir)
    serial = getattr(args, "serial", None) or DEFAULT_SERIAL
    return paths.latest_calibration(serial)


def _load_matching_calibration(model, path: Path):
    """Load a calibration and refuse one made for another hardware model."""
    calibration = load_calibration(path)
    if calibration.model_version != model.model_version:
        print(
            f"Calibration {path} was made for model "
            f"{calibration.model_version!r}, not {model.model_version!r}. "
            "Run `lamain calibrate`."
        )
        return None
    if calibration.servo_model and calibration.servo_model != model.servo.model:
        print(
            f"Calibration {path} was made for servo "
            f"{calibration.servo_model!r}, not {model.servo.model!r}. "
            "Run `lamain calibrate`."
        )
        return None
    return calibration


def cmd_studio(args) -> int:
    model = _model(args)
    calib = _resolve_calibration(args)
    if calib is None:
        print("No calibration found. Run `lamain calibrate` first.")
        return 1
    if _load_matching_calibration(model, calib) is None:
        return 1
    bus = _bus(args, model)
    keys = KeyReader()
    studio = Studio(
        model=model,
        bus=bus,
        calibration_path=calib,
        gestures=GestureStore(args.gesture_dir or DEFAULT_GESTURE_DIR),
        episodes=EpisodeStore(args.episode_dir or DEFAULT_EPISODE_DIR),
        keys=keys,
        step_deg=args.step,
    )
    try:
        studio.run()
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except Exception as exc:
        print(f"\nBUS/HARDWARE error: {exc}")
        print("Check the 6 V power and the USB cable, then retry.")
        return 1
    finally:
        keys.close()
        bus.close()


def cmd_demo_play(args) -> int:
    model = _model(args)
    calib = _resolve_calibration(args)
    if calib is None:
        print("No calibration found. Run `lamain calibrate` first.")
        return 1
    calibration = _load_matching_calibration(model, calib)
    if calibration is None:
        return 1
    store = EpisodeStore(args.episode_dir or DEFAULT_EPISODE_DIR)
    gestures = GestureStore(args.gesture_dir or DEFAULT_GESTURE_DIR)
    if args.name in store.list():
        steps = store.load(args.name).steps
        loops = args.loops
    else:
        if not gestures.exists(args.name):
            print(f"Not found (episode or gesture): {args.name}")
            return 1
        steps = [Step("gesture", None, name=args.name)]
        loops = args.loops or 1
    bus = _bus(args, model)
    controller = HandController(bus, model, calibration)
    controller.enable_torque(args.torque)
    player = Player(
        controller,
        PlayOptions(loops=loops, speed=args.speed, max_step_deg=90.0),
    )
    print(f"Playing '{args.name}' from zero (Ctrl-C to stop)...")
    try:
        player.play(steps, gestures=gestures)
        return 0
    except PlayerStopped as exc:
        print(f"Stopped: {exc}")
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except Exception as exc:
        print(f"\nBUS/HARDWARE error: {exc}")
        return 1
    finally:
        controller.disable_torque()
        bus.close()


def cmd_gesture_list(args) -> int:
    store = GestureStore(args.gesture_dir or DEFAULT_GESTURE_DIR)
    names = store.list()
    print("\n".join(names) if names else "(no gestures)")
    return 0


def cmd_episode_list(args) -> int:
    store = EpisodeStore(args.episode_dir or DEFAULT_EPISODE_DIR)
    names = store.list()
    print("\n".join(names) if names else "(no episodes)")
    return 0


def cmd_gesture_delete(args) -> int:
    store = GestureStore(args.gesture_dir or DEFAULT_GESTURE_DIR)
    existed = store.exists(args.name)
    store.delete(args.name)
    print("Deleted." if existed else "Not found.")
    return 0


def cmd_gesture_rename(args) -> int:
    store = GestureStore(args.gesture_dir or DEFAULT_GESTURE_DIR)
    print("Renamed." if store.rename(args.old, args.new) else "Failed.")
    return 0


def cmd_gesture_show(args) -> int:
    store = GestureStore(args.gesture_dir or DEFAULT_GESTURE_DIR)
    if not store.exists(args.name):
        print("Not found.")
        return 1
    print(json.dumps(store.load(args.name).to_dict(), indent=2))
    return 0


def cmd_episode_delete(args) -> int:
    store = EpisodeStore(args.episode_dir or DEFAULT_EPISODE_DIR)
    print("Deleted." if store.delete(args.name) else "Not found.")
    return 0


def cmd_episode_rename(args) -> int:
    store = EpisodeStore(args.episode_dir or DEFAULT_EPISODE_DIR)
    print("Renamed." if store.rename(args.old, args.new) else "Failed.")
    return 0


def cmd_episode_show(args) -> int:
    store = EpisodeStore(args.episode_dir or DEFAULT_EPISODE_DIR)
    try:
        episode = store.load(args.name)
    except FileNotFoundError:
        print("Not found.")
        return 1
    for i, step in enumerate(episode.steps, 1):
        label = f"gesture:{step.name}" if step.type == "gesture" else "keypoint"
        print(f"  {i}) {label}")
    return 0


# --------------------------------------------------------------------------- #
# Diagnostic
# --------------------------------------------------------------------------- #
def _print_diag(diag, serial: str) -> None:
    lo, hi = diag.stop_low, diag.stop_high
    print(
        f"  mount={diag.mount_ticks}  stop_low={lo}  stop_high={hi}  "
        f"travel={diag.travel_deg:.1f} deg"
    )
    print(
        f"    low : {'SERVO END' if diag.low_is_servo_end else 'inside'}"
        f"  eeprom_min={diag.servo_min_limit}"
    )
    print(
        f"    high: {'SERVO END' if diag.high_is_servo_end else 'inside'}"
        f"  eeprom_max={diag.servo_max_limit}"
    )
    print(
        f"    load {diag.load_min}..{diag.load_max}  temp_max {diag.temp_max} C  "
        f"Vmin {diag.voltage_min:.1f}"
    )
    print(f"    verdict: {diag.verdict}")


def _write_inspect_summary(run_dir, diags, serial: str) -> None:
    (run_dir / "summary.json").write_text(
        json.dumps([d.to_dict() for d in diags], indent=2), encoding="utf-8"
    )
    lines = [f"Inspection {serial}", ""]
    for d in diags:
        lines.append(
            f"{d.name:16} servo {d.servo_id}: stops=[{d.stop_low},{d.stop_high}] "
            f"travel={d.travel_deg:.1f} deg  load {d.load_min}..{d.load_max}  "
            f"temp {d.temp_max} C  V {d.voltage_min:.1f}"
        )
        lines.append(
            f"    low={'SERVO END' if d.low_is_servo_end else 'inside'}  "
            f"high={'SERVO END' if d.high_is_servo_end else 'inside'}  {d.verdict}"
        )
    (run_dir / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _inspect_jog(args, model, bus) -> int:
    """Manual jog to feel whether a blockage is mechanical. Reuses the studio key
    mapping (digits 0-9) and the same KeyReader."""
    torque = args.torque if args.torque is not None else model.calibration.probe_torque_pct
    step = (
        args.jog_step
        if args.jog_step is not None
        else max(1, model.servo.resolution // 128)
    )
    for j in model.joints.values():
        bus.set_torque_limit_pct(j.servo_id, torque)
        bus.set_goal_speed(j.servo_id, model.safety.move_speed)
        bus.set_torque_enable(j.servo_id, True)

    keys = KeyReader()
    print(f"Jog mode (torque {torque:.0f}%, step {step} ticks). Ctrl-C or 'q' to quit.")
    print("  0/1 index   2/3 middle   4/5 abduction   6/7 thumb base   8/9 thumb flex")
    print("  'l' = read all positions/loads, 'q' = quit.")
    try:
        while True:
            key = keys.read_key()
            if key is None:
                continue
            if key == "q":
                break
            if key == "l":
                for j in model.joints.values():
                    print(
                        f"  {j.name:16} pos={bus.read_position(j.servo_id):4} "
                        f"load={bus.read_load(j.servo_id):5}"
                    )
                continue
            if key in DEFAULT_KEYMAP:
                name, sign = DEFAULT_KEYMAP[key]
                j = model.joint(name)
                start = bus.read_position(j.servo_id)
                target = max(
                    model.servo.tick_min,
                    min(model.servo.tick_max, start + sign * step),
                )
                bus.write_goal(j.servo_id, target)
                bus.step()
                if not bus.simulated:
                    time.sleep(0.15)
                now = bus.read_position(j.servo_id)
                load = bus.read_load(j.servo_id)
                flag = "" if abs(now - target) <= 3 else "  <- blocked?"
                print(
                    f"  {name:16} {start:4} -> goal {target:4} "
                    f"(now {now:4}, load {load:5}){flag}"
                )
            else:
                print(f"(unused key {key!r})")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        keys.close()
        for j in model.joints.values():
            try:
                bus.set_torque_enable(j.servo_id, False)
            except Exception:
                pass
    return 0


def cmd_inspect(args) -> int:
    model = _model(args)
    bus = _bus(args, model)
    print("Resetting to the initial zero...")
    _home_all(bus, model)
    if args.jog:
        try:
            return _inspect_jog(args, model, bus)
        finally:
            bus.close()
    names = [args.joint] if args.joint else list(model.joints)
    serial = args.serial or DEFAULT_SERIAL
    run_dir = _new_run_dir("inspect", serial, args.logs_dir)
    step = args.step if args.step is not None else max(1, model.servo.resolution // 256)
    end_margin = (
        args.end_margin
        if args.end_margin is not None
        else max(2, round(20 * model.servo.resolution / 1024))
    )
    try:
        diags = []
        for name in names:
            joint = model.joint(name)
            print(f"\n=== inspect {name} (servo {joint.servo_id}) === (Ctrl-C to stop)")
            diag = sweep_joint(
                bus,
                model,
                joint,
                step_ticks=step,
                torque_pct=args.torque,
                settle_s=args.settle,
                endpoint_margin=end_margin,
            )
            _print_diag(diag, serial)
            diag_write_csv(diag, run_dir / f"{name}.csv")
            if args.plot:
                diag_plot(diag, run_dir / f"{name}.png")
            diags.append(diag)

        _write_inspect_summary(run_dir, diags, serial)
        print(f"\nSorties -> {run_dir}/  (csv, plot, summary.txt, summary.json)")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except Exception as exc:
        print(f"\nBUS/HARDWARE error: {exc}")
        print("Check the 6 V power and the USB cable, then retry.")
        return 1
    finally:
        for j in model.joints.values():
            try:
                bus.set_torque_enable(j.servo_id, False)
            except Exception:
                pass
        bus.close()


# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="lamain", description="LaMain - CLI")
    ap.add_argument("--model", default=None, help="path to hand_model.yaml")
    ap.add_argument("--port", default=None, help="serial port (default from the model)")
    ap.add_argument("--simulate", action="store_true", help="use the FakeBus")
    sub = ap.add_subparsers(dest="command", required=True)

    b = sub.add_parser("bus", help="bus tools (F1)")
    b.add_argument("action", choices=["scan", "set-id", "dump"])
    b.add_argument("old", nargs="?", type=int)
    b.add_argument("new", nargs="?", type=int)
    b.add_argument("servo_id", nargs="?", type=int, default=1)
    b.set_defaults(func=cmd_bus)

    a = sub.add_parser("assemble", help="assembly pose (F2)")
    a.add_argument("--release", action="store_true")
    a.add_argument("--yes", action="store_true")
    a.set_defaults(func=cmd_assemble)

    c = sub.add_parser("calibrate", help="automatic calibration (F3/F4)")
    c.add_argument("--verify", action="store_true", help="compare against nominal")
    c.add_argument("--repeat", type=int, default=1, help="repeat N times (repeatability)")
    c.add_argument(
        "--batch",
        action="store_true",
        help="calibrate straight through without zeroing the other servos between joints",
    )
    c.add_argument("--serial", default=None, help="hand serial number")
    c.add_argument("--tag", default=None, help="output file suffix (e.g. after_thumb)")
    c.add_argument("--calib-dir", default=None)
    c.add_argument("--logs-dir", default=None)
    c.add_argument("--yes", action="store_true", help="do not ask for confirmation")
    c.set_defaults(func=cmd_calibrate)

    k = sub.add_parser("collect-collisions", help="collision pose collection (F7)")
    k.add_argument("--serial", default=DEFAULT_SERIAL)
    k.add_argument("--out", default=None)
    k.add_argument("--logs-dir", default=None)
    k.set_defaults(func=cmd_collect)

    s = sub.add_parser("studio", help="interactive gesture/episode studio")
    s.add_argument("--calibration", default=None,
                   help="calibration json (default: the most recent)")
    s.add_argument("--calib-dir", default=None)
    s.add_argument("--gesture-dir", default=None)
    s.add_argument("--episode-dir", default=None)
    s.add_argument("--step", type=float, default=3.0, help="jog step in degrees")
    s.set_defaults(func=cmd_studio)

    d = sub.add_parser("demo", help="replay a demo")
    dsub = d.add_subparsers(dest="demo_action", required=True)
    dp = dsub.add_parser("play", help="replay an episode (fast)")
    dp.add_argument("name")
    dp.add_argument("--loops", type=int, default=0, help="0 = infinite")
    dp.add_argument("--speed", type=float, default=2.0)
    dp.add_argument("--torque", default="default",
                    choices=["free", "default", "grasp"])
    dp.add_argument("--calibration", default=None)
    dp.add_argument("--calib-dir", default=None)
    dp.add_argument("--episode-dir", default=None)
    dp.add_argument("--gesture-dir", default=None)
    dp.set_defaults(func=cmd_demo_play)

    g = sub.add_parser("gesture", help="gesture library")
    gsub = g.add_subparsers(dest="gesture_action", required=True)
    gl = gsub.add_parser("list")
    gl.add_argument("--gesture-dir", default=None)
    gl.set_defaults(func=cmd_gesture_list)
    gs = gsub.add_parser("show")
    gs.add_argument("name")
    gs.add_argument("--gesture-dir", default=None)
    gs.set_defaults(func=cmd_gesture_show)
    gd = gsub.add_parser("delete")
    gd.add_argument("name")
    gd.add_argument("--gesture-dir", default=None)
    gd.set_defaults(func=cmd_gesture_delete)
    gr = gsub.add_parser("rename")
    gr.add_argument("old")
    gr.add_argument("new")
    gr.add_argument("--gesture-dir", default=None)
    gr.set_defaults(func=cmd_gesture_rename)

    e = sub.add_parser("episode", help="episodes")
    esub = e.add_subparsers(dest="episode_action", required=True)
    el = esub.add_parser("list")
    el.add_argument("--episode-dir", default=None)
    el.set_defaults(func=cmd_episode_list)
    es = esub.add_parser("show")
    es.add_argument("name")
    es.add_argument("--episode-dir", default=None)
    es.set_defaults(func=cmd_episode_show)
    ed = esub.add_parser("delete")
    ed.add_argument("name")
    ed.add_argument("--episode-dir", default=None)
    ed.set_defaults(func=cmd_episode_delete)
    er = esub.add_parser("rename")
    er.add_argument("old")
    er.add_argument("new")
    er.add_argument("--episode-dir", default=None)
    er.set_defaults(func=cmd_episode_rename)

    i = sub.add_parser("inspect", help="diagnostic sweep of a joint")
    i.add_argument("joint", nargs="?", default=None, help="joint name (default: all)")
    i.add_argument("--step", type=int, default=None,
                   help="sweep step in ticks (default: model resolution / 256)")
    i.add_argument("--torque", type=float, default=None,
                   help="torque %% (default: probe torque)")
    i.add_argument("--settle", type=float, default=None)
    i.add_argument("--end-margin", type=int, default=None,
                   help="ticks from the servo end counted as an internal end")
    i.add_argument("--plot", action=argparse.BooleanOptionalAction, default=True,
                   help="write a PNG plot (default: on; use --no-plot to skip)")
    i.add_argument("--jog", action="store_true",
                   help="manual keyboard jog instead of the sweep (feel the blockage)")
    i.add_argument("--jog-step", type=int, default=None,
                   help="ticks per jog key press (default: model resolution / 128)")
    i.add_argument("--serial", default=None)
    i.add_argument("--logs-dir", default=None)
    i.set_defaults(func=cmd_inspect)

    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "bus" and args.action == "set-id" and (args.old is None or args.new is None):
        print("usage: lamain bus set-id OLD NEW")
        return 2
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
