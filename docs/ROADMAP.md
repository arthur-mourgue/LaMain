# Roadmap

What is planned and not built yet. What already works is in the README; history
lives in `git log`.

## Milestones

- **v0.1** — tactile fingertip pad (also fits the stock SO-101 gripper) +
  single-finger tracer bullet.
- **v0.3** — three-finger hand.
- **v1.0** — full hand + benchmark + cross-instance transfer result.

## Tactile pad (v0.1 headline)

Hardware and software not started. Magnets plus magnetometer planned; see
bench test B2 below.

## URDF generator

The URDF will be generated from the `hand_model.yaml` bundled in `lamain-core`
and must never be edited by hand (see [D-005](decisions/D-005-single-source-of-truth.md)).
Generator not written yet.

## LeRobot integration

A separate package `lerobot_robot_lamain`, depending on `lamain-core`. The
`lerobot_robot_` prefix is required for LeRobot plugin auto-discovery, so the
hand is used with `--robot.type=lamain`.

TODO: map each LeRobot motor/feature name to the lamain-core joint names
(`index_flex`, `middle_flex`, `abduction`, `thumb_rot`, `thumb_flex`).

## Benchmark tasks

Selection rule: a task enters the benchmark only if (1) the stock gripper fails
visibly, (2) success is binary, (3) a published comparable exists, or the
absence is stated. Protocol: every task is run with tactile, without tactile,
and with the stock SO-101 gripper; the number of rollouts is fixed before
running.

| # | Task | Proves | Published comparable |
|---|---|---|---|
| 1 | Pick-and-place, unknown mass | tactile | TacO arXiv:2605.21976; AnySkin arXiv:2409.08276 |
| 2 | Pick a flat object from a table | three fingers | none known |
| 3 | Plug insertion | tactile + industrial | eFlesh arXiv:2506.09994; T3 arXiv:2406.13640; TacO |
| 4 | In-hand reorientation | three fingers | TacO |
| 5 | Peg-in-hole, ~1 mm clearance | industrial | NIST Assembly Task Boards; arXiv:2406.05331 |

## Bench tests (no results yet)

- B1 Servo bus and power integration with the SO-101
- B2 Magnetic interference servo / magnetometer at 15, 25, 40 mm (idle, slow, fast, stall)
- B3 Servo torque on a 62 mm lever: stall, continuous, surface temperature after a 10 min hold
- B4 Static moment at the SO-101 flange
- B5 Four-bar linkage range 0-95 deg, coupling k close to 1
- B6 Cross-instance transfer: train on LM-0001, evaluate on LM-0002

## Hardware

CAD rev A not published yet. Hardware license CERN-OHL-P-2.0 proposed, pending
the co-designer's written agreement (see
[D-003](decisions/D-003-licences.md)).