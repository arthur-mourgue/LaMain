import json

from lamain_cli.main import main


def test_calibrate_simulated_writes_json(tmp_path):
    code = main(
        [
            "--simulate",
            "calibrate",
            "--serial",
            "TEST",
            "--calib-dir",
            str(tmp_path),
            "--logs-dir",
            str(tmp_path),
            "--yes",
        ]
    )
    assert code == 0
    data = json.loads((tmp_path / "TEST.json").read_text())
    assert data["hand_serial"] == "TEST"
    assert data["model_version"] == "lamain-v2-sts3036"
    assert data["servo_model"] == "STS3036"
    assert len(data["joints"]) == 5
    logs = list(tmp_path.glob("calibration/*_TEST/sweep.csv"))
    assert logs, "expected a segregated calibration sweep log"


def test_bus_set_id_requires_arguments(capsys):
    code = main(["bus", "set-id"])
    assert code == 2


def test_assemble_simulated(capsys):
    code = main(["--simulate", "assemble", "--release", "--yes"])
    assert code == 0
    out = capsys.readouterr().out
    assert "Positions AFTER" in out
    assert "[OK]" in out
