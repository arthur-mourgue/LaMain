from lamain_core.bus import FakeBus, FakeServo, move_to


def test_move_to_ramps_a_large_jump():
    # The STS3036 ignores a direct goal jump > ~1000 ticks; move_to must step.
    s = FakeServo(1, stop_min=0, stop_max=1023, position=0, goal=0, speed_ticks=50)
    bus = FakeBus([s])
    assert move_to(bus, 1, 1000, max_step=100, timeout_s=5.0)
    assert bus.read_position(1) == 1000


def test_move_to_gives_up_on_a_blocked_joint():
    # A blocked joint must not be pushed until the timeout: that is what
    # overheated the abduction servo during the reset.
    s = FakeServo(1, stop_min=100, stop_max=100, position=100, goal=100,
                  speed_ticks=50)
    bus = FakeBus([s])
    reads = {"n": 0}
    real = bus.read_position

    def counting(servo_id):
        reads["n"] += 1
        return real(servo_id)

    bus.read_position = counting
    assert not move_to(bus, 1, 1000, max_step=100, timeout_s=30.0)
    assert reads["n"] <= 25  # gave up on stall, not on the 30 s timeout


def test_move_to_accepts_the_servo_deadband():
    # The position loop settles a couple of ticks short of the goal: demanding
    # 1 tick reported a healthy middle_flex (2046 for 2048) as stuck.
    s = FakeServo(1, stop_min=0, stop_max=2046, position=2046, goal=2046,
                  speed_ticks=50)
    bus = FakeBus([s])
    assert not move_to(bus, 1, 2048, max_step=100, timeout_s=5.0)
    assert move_to(bus, 1, 2048, max_step=100, timeout_s=5.0, tolerance=8)


def test_servo_moves_and_clamps_at_stop():
    s = FakeServo(1, stop_min=100, stop_max=500, position=300, goal=300, speed_ticks=50)
    bus = FakeBus([s])
    bus.write_goal(1, 1000)  # beyond the high stop
    for _ in range(20):
        bus.step()
    assert bus.read_position(1) == 500
    assert bus.read_load(1) > 0


def test_load_is_low_when_free():
    s = FakeServo(1, stop_min=0, stop_max=1023, position=511, goal=511, speed_ticks=300)
    bus = FakeBus([s])
    bus.write_goal(1, 700)
    bus.step()
    assert bus.read_position(1) == 700
    assert bus.read_load(1) < 150


def test_absent_servo_raises():
    bus = FakeBus([FakeServo(1, 0, 1023, fault="absent")])
    assert bus.scan() == {}
    try:
        bus.read_position(1)
    except Exception as exc:
        assert "not on the bus" in str(exc)
    else:  # pragma: no cover
        raise AssertionError("expected an error")
