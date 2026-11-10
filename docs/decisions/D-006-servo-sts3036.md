# D-006: switch the design to Feetech STS3036 servos

- Date: 2026-11-09
- Status: Accepted

## Decision

The LaMain design now uses five Feetech **STS3036** servos instead of the
SCS0009. The SCS0009 version is archived at the `scs0009-final` git tag.

The code keeps a single, data-driven servo abstraction (`ServoSpec` in
`lamain_core.units`). The hand model describes the servo family, resolution,
mechanical range and register quirks; the bus selects the rustypot controller
from that description. The SCS0009 defaults remain known, so an SCS model file
restored from the archive can still be used with `--model`.

## Why

- The STS3036 is available, 6 V, 360°, 4096 ticks (0.088°/tick) and has
  substantially more torque than the SCS0009 (4.5 kg·cm stall vs ~1.5).
- It speaks the same Feetech TTL bus and is supported by rustypot 1.12+ under
  the STS3215 control table (model number 521).
- The hand mechanics are unchanged; the servo bodies fit the rev A mounts and
  the horns are mounted at the electrical zero (2048).

## Consequences

- `hand_model.yaml` is versioned `lamain-v2-sts3036`, with `assembly_position:
  2048` (0 rad) for every joint.
- Calibration files now record `servo_model`. A calibration whose
  `model_version` or `servo_model` does not match the model is refused instead
  of being applied to the wrong hardware. The old `LM-0001_9.json` is an
  SCS0009 calibration and is kept as a historical record only.
- Calibration tick thresholds are scaled ×4 (8/32/12/80/160/8 ticks).
- Because the STS3036 has more torque at the same percentage, the starting
  limits are 20 % free/default, 40 % grasp and 60 % calibration probe. They
  must be validated on the bench before being raised.
- Speeds are expressed in the model's unit: `steps_per_s` on the STS3036
  (`move_speed: 500`, about 44°/s) and the old SCS speed step on SCS0009.
- The STS3036 `acceleration` register is written before each speed
  (`acceleration: 50`): on this firmware, 0 leaves the servo motionless even
  with torque enabled and a reachable goal (verified on the bench).
- The firmware also ignores a direct goal jump larger than about 1000 ticks
  (one quarter turn). All moves are therefore ramped by `bus.move_to`, at most
  `resolution / 12` ticks per step (341 ticks on the STS3036); calibration,
  diagnostics, `assemble` and the home sequence use it.
- The bus selects `Sts3215PyController` or `Scs0009PyController` from the
  model; no call site hard-codes a servo family.

## Alternatives

- Keep the SCS0009 design: rejected because the servos are no longer the
  hardware being built.
- Maintain two parallel code paths: rejected; one model-driven path plus the
  archive tag is enough. The SCS0009 defaults stay available for restoration.

## Revisit if

- The STS3036 is replaced by another family: add a `ServoSpec` block to
  `hand_model.yaml`, no code change.
- The archived SCS0009 design must be maintained actively: restore its model
  file from `scs0009-final` and run the SCS calibration.
