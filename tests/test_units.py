from lamain_core.units import (
    ServoSpec,
    deg_to_ticks,
    rad_to_ticks,
    settle_tolerance,
    ticks_to_deg,
    ticks_to_rad,
)

SCS = ServoSpec(
    model="SCS0009",
    controller="Scs0009PyController",
    resolution=1024,
    range_deg=300.0,
)
STS = ServoSpec(
    model="STS3036",
    controller="Sts3215PyController",
    resolution=4096,
    range_deg=360.0,
    torque_register="torque_limit",
    torque_off=0,
    speed_unit="steps_per_s",
)


def test_scs_roundtrip_ticks_deg():
    assert abs(ticks_to_deg(deg_to_ticks(90.0, SCS), SCS) - 90.0) < 0.5


def test_scs_deg_per_tick_matches_spec():
    # 300 deg / 1024 steps = 0.293 deg
    assert abs(SCS.deg_per_tick - 0.29296875) < 1e-9
    assert SCS.tick_max == 1023
    assert SCS.mid_tick == 512


def test_sts_deg_per_tick_matches_spec():
    # 360 deg / 4096 steps = 0.087890625 deg
    assert abs(STS.deg_per_tick - 0.087890625) < 1e-9
    assert STS.tick_max == 4095
    assert STS.mid_tick == 2048


def test_rad_matches_ticks_on_both_families():
    for servo in (SCS, STS):
        rad = ticks_to_rad(rad_to_ticks(0.5, servo), servo)
        assert abs(rad - 0.5) < 0.01


def test_full_turn_maps_to_the_servo_resolution():
    assert deg_to_ticks(SCS.range_deg, SCS) == SCS.resolution
    assert deg_to_ticks(STS.range_deg, STS) == STS.resolution


def test_settle_tolerance_scales_with_the_servo():
    # 2 ticks at SCS resolution: the position loop deadband, scaled up.
    assert settle_tolerance(SCS) == 2
    assert settle_tolerance(STS) == 8
