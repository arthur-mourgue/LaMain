# LaMain

Open-source three-finger hand with tactile sensing that replaces the SO-101 gripper. Five Feetech STS3036 servos.

## Commands

- Check (lint, format, types, tests): `./scripts/check.sh`
- Tests only: `uv run pytest -x -q`

## Hardware safety: non-negotiable

- Ask before running anything that talks to the real hand: any `lamain` command other than `--help`.
- Every servo command goes through the safety filter. Never bypass it or add a path around it.
- Never change a joint range, a torque setting or a safety limit without asking.
- Hardware is reached only through `ServoBus`. All logic and tests run on `FakeBus`, with a fake clock instead of `sleep`.
- Torque is released on exit and on error. New motion code starts at reduced torque.

## Conventions specific to this project

- Units in names: `angle_rad`, `position_ticks`, `force_n`, `period_s`. Convert only through `lamain_core.units`.
- Geometry and hardware values come from the hand model YAML, never hard-coded.
- `lamain-core` stays dependency-light and imports no other package of the repo. Each new feature
  (retargeting, tactile, LeRobot plugin) is its own package under `packages/`.
- Calibration, gesture and episode files are a public format: changing one needs a migration and a test.
