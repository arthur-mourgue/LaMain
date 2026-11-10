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
        "abduction",
        "index_flex",
        "middle_flex",
        "thumb_rot",
        "thumb_flex",
    )
    assert m.joint("abduction").servo_id == 1
    assert m.joint("index_flex").servo_id == 2
    assert m.joint("middle_flex").servo_id == 3
    assert m.joint("thumb_rot").servo_id == 4
    assert m.joint("thumb_flex").servo_id == 5
    assert m.joint("abduction").calibration_torque_pct == 60
    assert m.joint("abduction").probe_torque_pct == 80
    assert m.joint("index_flex").is_crank
    assert m.joint("abduction").joint_type == "two_stop"
    assert m.joint("thumb_rot").joint_type == "two_stop"


def test_fast_step_does_not_outrun_the_servo():
    # The goal advances by fast_step_ticks per read; the servo can only do
    # move_speed * settle_s. A bigger step makes the command run away from the
    # position (bad stop detection, arbitrary push at a stop).
    m = make_model()
    ticks_per_read = m.safety.move_speed * m.calibration.settle_s
    assert m.calibration.fast_step_ticks <= ticks_per_read


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
    for c in hand.joints.values():
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


def test_reset_holds_the_current_pose_before_enabling_torque():
    # The STS3036 ignores a goal more than ~1000 ticks away: the assembly zero
    # must be reached by ramping from the current pose, never by one direct
    # jump (818 -> 2048 = 1230 ticks left the real joint stuck).
    from lamain_core.calibration import _prepare_all

    m = make_model()
    start = 818
    servos = []
    for name in m.calibration_order:
        j = m.joint(name)
        if name == "abduction":
            # stops measured by `lamain inspect` on the bench
            servos.append(
                FakeServo(
                    j.servo_id,
                    stop_min=379,
                    stop_max=2821,
                    position=start,
                    goal=start,
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
    writes: list[tuple[int, int]] = []
    real = bus.write_goal

    def recording(servo_id, ticks):
        writes.append((servo_id, ticks))
        real(servo_id, ticks)

    bus.write_goal = recording
    positions = _prepare_all(bus, m)

    abduction = m.joint("abduction").servo_id
    first = next(ticks for sid, ticks in writes if sid == abduction)
    assert first == start  # hold the current pose, then ramp
    zero = m.joint("abduction").assembly_position
    assert abs(positions["abduction"] - zero) <= units.settle_tolerance(m.servo)


def test_reset_accepts_a_servo_settling_a_few_ticks_short():
    # middle_flex settles 2 ticks short of its assembly zero on the bench
    # (2046 for 2048): that is the position loop deadband, not a jam.
    from lamain_core.calibration import _prepare_all

    m = make_model()
    servos = []
    for name in m.calibration_order:
        j = m.joint(name)
        half = units.deg_to_ticks(j.nominal_travel_deg / 2, m.servo)
        stop_min = max(m.servo.tick_min, j.assembly_position - half)
        stop_max = min(m.servo.tick_max, j.assembly_position + half)
        position = j.assembly_position
        if name == "middle_flex":
            stop_max = j.assembly_position - 2
            position = j.assembly_position - 16
        servos.append(
            FakeServo(
                j.servo_id,
                stop_min=stop_min,
                stop_max=stop_max,
                position=position,
                goal=position,
            )
        )
    bus = FakeBus(servos)
    positions = _prepare_all(bus, m)
    zero = m.joint("middle_flex").assembly_position
    assert positions["middle_flex"] > zero - 16  # it did move up
    assert abs(positions["middle_flex"] - zero) <= units.settle_tolerance(m.servo)


def test_calibration_aborts_when_the_assembly_zero_is_unreachable():
    # A joint that cannot come back to its assembly zero must stop the run
    # instead of being pushed there for seconds (that overheats the servo).
    import pytest
    from lamain_core.calibration import CalibrationAbort

    m = make_model()
    stuck = 3873  # same pose as the jammed abduction on the bench
    servos = []
    for name in m.calibration_order:
        j = m.joint(name)
        if name == "abduction":
            servos.append(
                FakeServo(
                    j.servo_id,
                    stop_min=stuck,
                    stop_max=stuck,
                    position=stuck,
                    goal=stuck,
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
    with pytest.raises(CalibrationAbort, match="assembly zero 2048 unreachable"):
        calibrate_hand(bus, m, "TEST", logs_dir=None)


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


def test_slow_free_joint_reaches_the_servo_end_instead_of_aborting():
    # A free joint must still be scanned to the end when the servo is slow:
    # the read budget has to follow move_speed * settle_s, not the fast step.
    m = make_model()
    j = m.joint("index_flex")
    slow = int(m.safety.move_speed * m.calibration.settle_s)  # ticks per read
    bus = FakeBus(
        [
            FakeServo(
                j.servo_id,
                stop_min=m.servo.tick_min,
                stop_max=m.servo.tick_max,
                position=m.servo.mid_tick,
                goal=m.servo.mid_tick,
                speed_ticks=slow,
            )
        ]
    )
    from lamain_core.calibration import CalibrationLog, calibrate_joint

    cal = calibrate_joint(bus, j, m, CalibrationLog())
    assert cal.status == "failed"
    assert cal.low_at_servo_end and cal.high_at_servo_end


def test_approach_budget_follows_the_servo_speed():
    # Even with a fast step much larger than what the servo can do per read,
    # the scan must be allowed to cross the whole travel.
    from lamain_core import calibration as cal_mod

    m = make_model()
    j = m.joint("index_flex")
    slow = int(m.safety.move_speed * m.calibration.settle_s)  # ticks per read
    assert slow < 32
    bus = FakeBus(
        [
            FakeServo(
                j.servo_id,
                stop_min=m.servo.tick_min,
                stop_max=m.servo.tick_max,
                position=m.servo.tick_min,
                goal=m.servo.tick_min,
                speed_ticks=slow,
            )
        ]
    )
    res = cal_mod._approach(bus, j, m, +1, 32, cal_mod.CalibrationLog())
    assert res.at_servo_end
    assert res.ticks >= m.servo.tick_max - m.calibration.stable_epsilon_ticks


def test_stop_at_the_command_boundary_is_confirmed(monkeypatch):
    # When the goal is clamped at tick_min, a stalling position must still be
    # pushed at probe torque instead of being accepted straight away.
    from lamain_core import calibration as cal_mod

    m = make_model()
    j = m.joint("abduction")
    stuck = 100  # far from the servo end, the goal reaches tick_min first
    bus = FakeBus(
        [
            FakeServo(
                j.servo_id,
                stop_min=stuck,
                stop_max=stuck,
                position=stuck,
                goal=stuck,
                speed_ticks=8,
            )
        ]
    )
    calls: list[int] = []
    real = cal_mod._confirm_stop

    def spy(bus_, joint_, model_, direction_, pos_, log_, trace=None):
        calls.append(pos_)
        return real(bus_, joint_, model_, direction_, pos_, log_, trace)

    monkeypatch.setattr(cal_mod, "_confirm_stop", spy)
    res = cal_mod._approach(bus, j, m, -1, 32, cal_mod.CalibrationLog())
    assert calls, "stop accepted at the command boundary without a torque probe"
    assert res.ticks == stuck


def test_aborted_calibration_keeps_the_sweep_log(tmp_path):
    import pytest
    from lamain_core.calibration import CalibrationAbort

    m = make_model()
    bus = FakeBus.standard_hand(m)
    calls = {"n": 0}

    def spiking_temperature(servo_id):
        calls["n"] += 1
        # preflight reads every servo once; overheat on the first sweep read
        return 30 if calls["n"] <= len(m.joints) else 70

    bus.read_temperature = spiking_temperature
    with pytest.raises(CalibrationAbort):
        calibrate_hand(bus, m, "TEST", logs_dir=tmp_path)

    lines = (tmp_path / "sweep.csv").read_text(encoding="utf-8").splitlines()
    assert len(lines) > 1  # header + at least one sample

