import math

from lamain_core.bus import FakeBus
from lamain_core.calibration import calibrate_hand
from lamain_core.controller import HandController
from lamain_core.jog import JogController
from lamain_core.model import default_model_path, load_hand_model


def _controller():
    m = load_hand_model(default_model_path())
    bus = FakeBus.standard_hand(m)
    hand = calibrate_hand(bus, m, "TEST")
    ctrl = HandController(bus, m, hand)
    ctrl.enable_torque("free")
    return m, ctrl


def test_key_step_is_fixed_and_signed():
    m, ctrl = _controller()
    jog = JogController(ctrl, step_deg=3.0)
    before = jog.q["index_flex"]
    assert jog.handle_key("1")
    after = jog.q["index_flex"]
    assert after > before
    span_deg = ctrl.q_span_deg("index_flex", +1)
    assert math.isclose(after - before, 3.0 / span_deg, rel_tol=0.05)
    assert jog.handle_key("0")
    assert jog.q["index_flex"] < after


def test_unmapped_key_is_ignored():
    m, ctrl = _controller()
    jog = JogController(ctrl)
    assert not jog.handle_key("z")


def test_jog_stays_within_bounds():
    m, ctrl = _controller()
    jog = JogController(ctrl, step_deg=5.0)
    for _ in range(200):
        jog.handle_key("1")   # push index flexion hard
    for _ in range(300):
        ctrl.bus.step()
    lim = ctrl.safety.limits["index_flex"]
    ticks = ctrl.bus.read_position(ctrl.model.joint("index_flex").servo_id)
    assert ticks <= lim.max_ticks
    assert jog.q["index_flex"] <= 1.0 + 1e-6
    assert ticks == lim.max_ticks


def test_arrow_keys_jog_the_thumb():
    m, ctrl = _controller()
    jog = JogController(ctrl, step_deg=3.0)
    before_flex = jog.q["thumb_flex"]
    assert jog.handle_key("up")
    after_up = jog.q["thumb_flex"]
    assert after_up > before_flex
    assert jog.handle_key("down")
    assert jog.q["thumb_flex"] < after_up
    before_rot = jog.q["thumb_rot"]
    assert jog.handle_key("right")
    after_right = jog.q["thumb_rot"]
    assert after_right > before_rot
    assert jog.handle_key("left")
    assert jog.q["thumb_rot"] < after_right


def test_capture_roundtrip():
    m, ctrl = _controller()
    jog = JogController(ctrl)
    jog.handle_key("3")
    pose = jog.capture()
    assert set(pose.q) == set(ctrl.joint_names)
    assert set(pose.rad) == set(ctrl.joint_names)
