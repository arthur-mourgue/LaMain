# LaMain

An open, low-cost three-finger hand for the SO-101 arm.

<p align="center">
  <img src="docs/lamain.gif" alt="LaMain prototype in motion" width="360">
</p>

**Early prototype under active development.** No tactile sensor is fitted, and no
dataset or policy exists. Interfaces and hardware will change.

## Why

Tactile data is scarce. Most hands that can produce it are expensive or closed.
Learning-based manipulation needs many cheap comparable devices to collect that data.
LaMain is one. It is a 3D-printed hand that mounts on an SO-101 arm and runs on hobby
servos. Tactile sensing is the next milestone. The target is a hand under 200 g and
under 200 EUR.

## How it works

Five Feetech STS3036 servos drive `index_flex`, `middle_flex`, `abduction`, `thumb_rot`
and `thumb_flex` through rigid linkages. Geometry and joint limits live in
`hand_model.yaml`, the single source of truth for code, calibration and the future URDF.

All commands pass a safety filter with range, step and torque limits. Calibration
records joint limits from the hand posed at released torque. Gestures and episodes are
plain JSON files.

```bash
lamain calibrate      # record joint limits
lamain studio         # jog, record gestures and episodes
lamain inspect        # per-joint diagnostic sweep
```

Tests run on a simulated bus with a fake clock. No hardware is needed.

## Status

| | |
|---|---|
| Hardware | revision A, five STS3036 servos, SO-101 mount |
| Software | bus, calibration, safety filter, controller, studio, episodes |
| Tests | full suite on `FakeBus`, run in CI |
| Tactile sensing | not started |
| LeRobot plugin | not started |
| Dataset and benchmark | not started |

## Quickstart

```bash
uv sync                 # install the workspace and dependencies
uv run lamain --help    # CLI overview
uv run pytest           # tests on FakeBus, no hardware
```

## Repository layout

```
lamain/
├── packages/lamain-core/   # library: bus, model, calibration, safety, controller, demos
├── packages/lamain-cli/    # the `lamain` CLI
├── hands/LM-0001/          # per-hand build manifest and calibration files
├── demos/                  # gestures and episodes
├── docs/                   # roadmap and decision records
└── tests/                  # FakeBus tests, run in CI
```

## Hardware

The bill of materials is in a [shared spreadsheet](https://docs.google.com/spreadsheets/d/1QRkrFNEr4OLIYoRjOxhvR7IqRy3qPOLTq45OjTfyoFY/edit?usp=sharing).
CAD rev A is not published. The SCS0009 design is kept at the `scs0009-final` tag. The
switch to STS3036 is recorded in [D-006](docs/decisions/D-006-servo-sts3036.md).

## Planned

Tactile fingertip pad, URDF generation, LeRobot plugin, then a reproducible benchmark
against the stock gripper. Open bench tests are in [docs/ROADMAP.md](docs/ROADMAP.md).

## License

Software is Apache-2.0, see [LICENSE](LICENSE). The hardware licence is pending, see
[D-003](docs/decisions/D-003-licences.md).

## Credits

Arthur Mourgue and Julien Navet for the CAD. Inspired by Pollen Robotics'
[AmazingHand](https://github.com/pollen-robotics/AmazingHand).
