import pytest

from lamain_core import units
from lamain_core.bus import FakeBus
from lamain_core.calibration import HandCalibration, JointCalibration, calibrate_hand
from lamain_core.controller import HandController
from lamain_core.model import default_model_path, load_hand_model


def _controller():
    m = load_hand_model(default_model_path())
    bus = FakeBus.standard_hand(m)
    hand = calibrate_hand(bus, m, "TEST")
    ctrl = HandController(bus, m, hand)
    ctrl.enable_torque()
    return m, ctrl


def test_rad_ticks_roundtrip():
    m, ctrl = _controller()
    for name in ctrl.joint_names:
        rad = 0.3
        ticks = ctrl._rad_to_ticks(name, rad)
        assert abs(ctrl._ticks_to_rad(name, ticks) - rad) < 0.01


def test_set_and_get_positions():
    m, ctrl = _controller()
    target = {name: 0.0 for name in ctrl.joint_names}
    applied = ctrl.set_joint_positions(target)
    assert set(applied) == set(ctrl.joint_names)
    # 0 rad = reference, which is inside the measured working range
    for name in ctrl.joint_names:
        assert abs(applied[name]) < 1e-6


def test_normalized_roundtrip():
    m, ctrl = _controller()
    applied = ctrl.set_normalized({"index_flex": 0.5})
    pos_ticks, _ = ctrl._spans_ticks("index_flex")
    span = units.ticks_to_rad(pos_ticks, m.servo)
    assert abs(applied["index_flex"] - 0.5 * span) < 1e-6


def _drive(ctrl, name, q):
    for _ in range(200):
        ctrl.set_normalized({name: q})
        for _ in range(2):
            ctrl.bus.step()


def test_q_reaches_both_measured_stops():
    m, ctrl = _controller()
    for name in ctrl.joint_names:
        lim = ctrl.safety.limits[name]
        _drive(ctrl, name, 1.0)
        assert ctrl.bus.read_position(ctrl.model.joint(name).servo_id) == lim.max_ticks
        _drive(ctrl, name, -1.0)
        assert ctrl.bus.read_position(ctrl.model.joint(name).servo_id) == lim.min_ticks


def test_refuses_invalid_calibration():
    m = load_hand_model(default_model_path())
    bad = HandCalibration(
        hand_serial="X",
        model_version=m.model_version,
        joints={
            "index_flex": JointCalibration(
                name="index_flex",
                servo_id=1,
                joint_type="crank",
                mount_ticks=0,
                reference_ticks=0,
                direction=1,
                stop_low_ticks=0,
                stop_high_ticks=0,
                min_ticks=0,
                max_ticks=0,
                home_ticks=0,
                status="failed",
            )
        },
    )
    with pytest.raises(RuntimeError):
        HandController(FakeBus([]), m, bad)


def _valid_joint(model) -> JointCalibration:
    mid = model.servo.mid_tick
    return JointCalibration(
        name="index_flex",
        servo_id=model.joint("index_flex").servo_id,
        joint_type="crank",
        mount_ticks=mid,
        reference_ticks=mid,
        direction=1,
        stop_low_ticks=mid - 200,
        stop_high_ticks=mid + 200,
        min_ticks=mid - 200,
        max_ticks=mid + 200,
        home_ticks=mid,
    )


def test_refuses_calibration_for_another_model():
    m = load_hand_model(default_model_path())
    calibration = HandCalibration(
        hand_serial="X",
        model_version="lamain-v1",
        joints={"index_flex": _valid_joint(m)},
    )
    with pytest.raises(RuntimeError, match="made for model"):
        HandController(FakeBus([]), m, calibration)


def test_refuses_calibration_for_another_servo():
    m = load_hand_model(default_model_path())
    calibration = HandCalibration(
        hand_serial="X",
        model_version=m.model_version,
        servo_model="SCS0009",
        joints={"index_flex": _valid_joint(m)},
    )
    with pytest.raises(RuntimeError, match="made for servo"):
        HandController(FakeBus([]), m, calibration)


def test_hardware_limits_skipped_on_fake():
    m, ctrl = _controller()
    ctrl.write_hardware_limits()  # must not raise on the FakeBus
