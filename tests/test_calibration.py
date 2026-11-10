from lamain_core import units
from lamain_core.bus import FakeBus, FakeServo
from lamain_core.calibration import calibrate_hand
from lamain_core.model import default_model_path, load_hand_model


def make_model():
    return load_hand_model(default_model_path())


def test_model_loads_all_joints():
    m = make_model()
    assert len(m.joints) == 5
    assert m.servo.model == "STS3036"
    assert m.servo.resolution == 4096
    assert m.servo.range_deg == 360.0
    assert m.servo.controller == "Sts3215PyController"
    assert m.servo.acceleration == 50
    assert all(j.assembly_position == m.servo.mid_tick for j in m.joints.values())
    assert m.calibration_order == (
        "index_middle_abd",
        "index_flex",
        "middle_flex",
        "thumb_rot",
        "thumb_flex",
    )
    assert m.joint("index_flex").servo_id == 2
    assert m.joint("middle_flex").servo_id == 3
    assert m.joint("index_middle_abd").servo_id == 1
    assert m.joint("thumb_rot").servo_id == 4
    assert m.joint("thumb_flex").servo_id == 5
    assert m.joint("index_flex").is_crank
    assert m.joint("index_middle_abd").joint_type == "two_stop"
    assert m.joint("thumb_rot").joint_type == "two_stop"


def test_calibrate_end_to_end_on_fakebus():
    m = make_model()
    bus = FakeBus.standard_hand(m)
    hand = calibrate_hand(bus, m, "TEST", logs_dir=None)
    assert hand.valid, {n: c.cause for n, c in hand.joints.items()}
    # cranks: dead center = mid travel = assembly position
    for name in ("index_flex", "middle_flex", "thumb_flex"):
        c = hand.joints[name]
        assert c.joint_type == "crank"
        assert abs(c.dead_center_ticks - m.joint(name).assembly_position) <= 3
    # every working range stays inside the measured stops
    for c in hand.joints.values():
        assert c.stop_low_ticks <= c.min_ticks <= c.max_ticks <= c.stop_high_ticks
        assert c.reference_ticks >= c.stop_low_ticks
        assert c.reference_ticks <= c.stop_high_ticks
    # two zeros stored, distinct from the reference for two_stop joints
    for c in hand.joints.values():
        assert c.mount_ticks > 0


def test_reference_is_middle_and_both_ways():
    m = make_model()
    bus = FakeBus.standard_hand(m)
    hand = calibrate_hand(bus, m, "TEST")
    for name, c in hand.joints.items():
        # reference sits between the two stops -> both directions available
        assert c.stop_low_ticks <= c.min_ticks <= c.reference_ticks
        assert c.reference_ticks <= c.max_ticks <= c.stop_high_ticks
        assert c.min_ticks < c.reference_ticks < c.max_ticks


def test_calibration_is_repeatable():
    m = make_model()
    bus = FakeBus.standard_hand(m)
    a = calibrate_hand(bus, m, "TEST")
    b = calibrate_hand(bus, m, "TEST")
    for name in a.joints:
        assert abs(a.joints[name].reference_ticks - b.joints[name].reference_ticks) <= 2


def test_blocked_joint_is_reported_failed():
    m = make_model()
    mid = m.servo.mid_tick

    servos = []
    for name in m.calibration_order:
        j = m.joint(name)
        if j.name == "index_flex":
            servos.append(
                FakeServo(
                    j.servo_id,
                    stop_min=mid,
                    stop_max=mid,
                    position=mid,
                    goal=mid,
                )
            )
        else:
            half = units.deg_to_ticks(j.nominal_travel_deg / 2, m.servo)
            servos.append(
                FakeServo(
                    j.servo_id,
                    stop_min=max(m.servo.tick_min, j.assembly_position - half),
                    stop_max=min(m.servo.tick_max, j.assembly_position + half),
                    position=j.assembly_position,
                    goal=j.assembly_position,
                )
            )
    bus = FakeBus(servos)
    hand = calibrate_hand(bus, m, "TEST")
    assert not hand.valid
    assert hand.joints["index_flex"].status == "failed"


def test_no_mechanical_stop_is_flagged():
    # free servo over the whole travel: both stops fall on the servo's internal
    # end -> calibration marked as failed.
    m = make_model()
    servos = [
        FakeServo(
            m.joint("index_flex").servo_id,
            stop_min=m.servo.tick_min,
            stop_max=m.servo.tick_max,
            position=m.servo.mid_tick,
            goal=m.servo.mid_tick,
            speed_ticks=max(1, m.servo.resolution // 25),
        )
    ]
    bus = FakeBus(servos)
    from lamain_core.calibration import CalibrationLog, calibrate_joint

    log = CalibrationLog()
    cal = calibrate_joint(bus, m.joint("index_flex"), m, log)
    assert cal.status == "failed"
    assert cal.low_at_servo_end and cal.high_at_servo_end
