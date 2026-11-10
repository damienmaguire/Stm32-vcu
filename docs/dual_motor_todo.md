# Dual-motor / PTO follow-up work

Planned follow-ups to the second-inverter feature (`Param::Inverter2`, `Param::Inverter2UseCase`,
`PTOControl` in [include/ptocontrol.h](../include/ptocontrol.h)/[src/ptocontrol.cpp](../src/ptocontrol.cpp)).
Not yet implemented - notes below are starting points, not final designs.

## 1. PTO control mode (`Param::PTOMode`)

Unifies what were originally two separate ideas (adjustable-via-pot, and speed-target control)
into one mode selector, since they're really two independent axes - *source* (toggle switch vs.
pot) and *target* (torque vs. speed) - not separate features:

```
Param::PTOMode: None, ToggleTorq, ToggleSpeed, PotTorque, PotSpeed
```

- **None** - PTO always outputs 0, regardless of `Param::Inverter2UseCase`/`PTOENABLE`/etc. Same
  "defense in depth" layered-default philosophy already used elsewhere (`Inverter2UseCase`
  defaults to `NotUsed`, `PTOTorque` defaults to 0) - nothing happens until every layer is
  deliberately configured.
- **ToggleTorq** - today's exact current behavior: the `PTOENABLE` pin (see
  `PTOControl::GetTorque()` in [src/ptocontrol.cpp](../src/ptocontrol.cpp)) gates a fixed
  `Param::PTOTorque`.
- **ToggleSpeed** - same `PTOENABLE` pin, but gates a fixed `Param::PTOSpeedTarget` through the
  closed-loop speed controller described below instead of a raw torque value.
- **PotTorque** - continuous control: an assigned `PTO_POT` analogue input (see below) scales
  linearly between `PTOTorqueMin` and `PTOTorqueMax`, instead of an on/off toggle.
  - The project already has an assignable analogue-input system for this:
    `IOMatrix::analoguepinfuncs` ([include/iomatrix.h](../include/iomatrix.h)), currently
    `PILOT_PROX`/`VAC_SENSOR`/`HEATER_POT`, backed by 2 assignable physical pins
    (`GPA1Func`/`GPA2Func` params, `AnaIn::GP_analog1`/`GP_analog2` -
    [src/iomatrix.cpp](../src/iomatrix.cpp)). Add `PTO_POT` the same way, appended at the end
    before `LAST_ANAL` (same saved-param backward-compatibility rule as `PTOENABLE`).
  - Raw pot calibration (the ADC min/max of the pot's physical travel) belongs to the *physical
    pin*, not the function assigned to it - see the calibration note below. `PTO_POT` itself only
    needs its own **output** range: `PTOTorqueMin`/`PTOTorqueMax` (a min *and* max, not just a
    max, so a nonzero baseline torque at the pot's lowest position is possible).
- **PotSpeed** - same `PTO_POT` pot, but scales between a `PTOSpeedMin`/`PTOSpeedMax` pair
  (analogous to the torque min/max) and drives the target through the closed-loop speed
  controller.
- **Analogue pin calibration (shared across whatever function is assigned)** - rather than every
  new pot-driven function inventing its own raw-ADC min/max pair (as originally sketched here),
  give each of the 2 general-purpose analogue pins its own calibration - e.g.
  `GPA1Min`/`GPA1Max`, `GPA2Min`/`GPA2Max` - reused by whichever function `GPA1Func`/`GPA2Func`
  currently has assigned. The raw pot wiring/ADC range is a property of the physical input, not of
  what it's currently mapped to, so this scales better as more analogue-mapped functions are added
  (2 pins × 2 params instead of 2 params duplicated per function) - only each function's *output*
  range (e.g. `PTOTorqueMin`/`Max`, item 3's ratio range) still needs its own params.
- **Closed-loop speed control** (needed for `ToggleSpeed`/`PotSpeed`): the `Inverter` interface
  only exposes `SetTorque()` - there's no `SetSpeed()` anywhere in this codebase - so hitting a
  speed target means a small controller *inside* `PTOControl` itself: read
  `selectedInverter2->GetMotorSpeed()` as feedback (already available - used today only for the
  `speed2` telemetry param), compare to the active speed target, and output a torque command that
  drives toward it. Start with a simple P or PI loop - nothing that elaborate exists elsewhere in
  this codebase to copy from.

## 2. Motor speed limiter (overspeed safety cutback)

Neither drive motor currently has any closed-loop protection against overspeed if it loses load
(a real risk for anything driven without a rigid/geared connection - e.g. a PTO-driven pump losing
fluid, or a wheel losing traction). If the actual RPM exceeds a safe limit, torque needs to be
cut back automatically rather than trusting the operator to notice.

- There's already a single-motor precedent: `Param::rpmlim` /
  `Throttle::speedLimit` ([src/throttle.cpp](../src/throttle.cpp), wired up in
  `Param::Change()` in [src/stm32_vcu.cpp](../src/stm32_vcu.cpp)) limits the *primary* motor via
  the normal throttle-processing path. That path isn't used at all for `PTOMotor` mode (item 1
  above) or for a from-scratch `Inverter2` torque command, so it doesn't currently protect
  inverter2.
- Needs its own feedback loop reading `selectedInverter2->GetMotorSpeed()` and cutting back
  whatever torque value is about to be sent, before the `SetTorque()` call in `Ms10Task`. A simple
  proportional taper as speed approaches a configurable limit (mirroring however `rpmlim` behaves
  for motor1) is likely enough to start; a hard cutoff is the minimum viable version.
- Shares plumbing with item 1's closed-loop speed control (both need a speed-feedback loop against
  `GetMotorSpeed()`) - worth designing them together rather than bolting on two separate feedback
  mechanisms.
- Decide scope: does this apply just to inverter2 (the newer, currently-unprotected path), or
  should it also become a general safety net for inverter1 layered on top of the existing
  `rpmlim` behavior?

## 3. Analogue input with direct control of `Inverter2TorqueRatio`

For `Inverter2UseCase = SecondDriveMotor`, let a dial control the motor1/motor2 torque split
(`Param::Inverter2TorqueRatio`, [src/stm32_vcu.cpp](../src/stm32_vcu.cpp)) live, instead of only
being settable via the web/laptop interface - e.g. a driver-adjustable front/rear or left/right
bias control.

- Same mechanism as item 1's `PTO_POT`: a new `IOMatrix::analoguepinfuncs` value (e.g.
  `TORQUESPLIT_POT`), assignable to one of the 2 general-purpose analogue pins via
  `GPA1Func`/`GPA2Func`. Raw pot calibration comes from that pin's own `GPA1Min`/`Max` or
  `GPA2Min`/`Max` (see item 1's calibration note) - no separate pot-calibration params needed here.
- Only needs its own **output** range, same pattern as `PTOTorqueMin`/`Max`:
  `TorqueSplitMin`/`TorqueSplitMax` mapping linearly to `Inverter2TorqueRatio`'s 0-100 range - or a
  narrower configurable sub-range, if it's worth letting the installer limit how far off-center
  the dial can bias the split.
- Only meaningful for `SecondDriveMotor`; should have no effect in `PTOMotor`/`NotUsed` modes
  (`Inverter2TorqueRatio` isn't read at all outside the `SecondDriveMotor` case in `Ms10Task`
  today).
