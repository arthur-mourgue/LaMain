from lamain_core.bus import FakeBus, FakeServo
from lamain_core.diagnostics import plot, sweep_joint, write_csv
from lamain_core.model import default_model_path, load_hand_model


def _model_bus():
    m = load_hand_model(default_model_path())
    return m, FakeBus.standard_hand(m)


def test_sweep_reports_full_travel():
    m, bus = _model_bus()
    d = sweep_joint(bus, m, m.joint("index_flex"), step_ticks=16)
    assert d.stop_low < m.servo.mid_tick
    assert d.stop_high > m.servo.mid_tick
    assert d.travel_deg > 250
    assert d.temp_max >= 0
    assert d.voltage_min > 0


def test_sweep_flags_a_servo_end():
    m = load_hand_model(default_model_path())
    # stop low inside the travel, stop high at the servo end
    bus = FakeBus(
        [
            FakeServo(
                4,
                stop_min=800,
                stop_max=m.servo.tick_max - 18,
                position=1000,
                goal=1000,
            )
        ]
    )
    d = sweep_joint(bus, m, m.joint("thumb_rot"), step_ticks=16)
    assert d.low_is_servo_end is False
    assert d.high_is_servo_end is True
    assert "internal end" in d.verdict


def test_outputs(tmp_path):
    m, bus = _model_bus()
    d = sweep_joint(bus, m, m.joint("index_flex"), step_ticks=16)
    csv_path = tmp_path / "d.csv"
    write_csv(d, csv_path)
    assert csv_path.exists()
    png_path = tmp_path / "d.png"
    plot(d, png_path)
    assert png_path.exists()
