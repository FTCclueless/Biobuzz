# Teleop Main controls

These controls follow the supplied Teleop sample. Teleop starts in automatic
shooter mode, as the sample does after its initialization loop.

| Gamepad | Button | Action |
| --- | --- | --- |
| 1 | Left bumper | Toggle intake on/off and reset to forward |
| 1 | A (without Start) | Start intake reversed from idle, otherwise toggle direction |
| 1 or 2 | Left trigger press above 0.2 | Toggle front/back intake side (starts at front) |
| 2 | A | Manual shooter mode, flywheel off |
| 2 | B (without Start) | Automatic shooter mode |
| 2 | Y | Manual mode with the sample's turret-tracking flag (turret unavailable) |
| 1, manual | B / Y / X | Close / middle / far shooter preset |
| 1, manual | Right trigger above 0.2, or hold B/Y/X for 500 ms | Flywheel off |
| 1, manual | Hold right bumper | Open blocker and feed intake; release stops intake and closes blocker |
| 1, automatic | Y | Request aim |
| 1, automatic | Right bumper press while READY | Request shooting; release cancels feeding and returns from SHOOT to aiming |
| 1, automatic | Right trigger above 0.2 | Request stop |
| 2, automatic | Hold right / left bumper | Force flywheel velocity to 580 / 400 (right takes priority) |

Both gamepads rumble on mode changes and when a selected manual preset reaches
speed or the automatic shooter becomes READY.

## Differences and limitations

- The turret is commented out in the current Shooter subsystem. Gamepad 2 Y
  retains the tracking flag but cannot move a turret. Turret-position telemetry
  from the sample is omitted.
- The sample's `light0G` and `light0P` hardware is absent. Speed feedback uses
  rumble and telemetry instead.
- Automatic IDLE targets the CLOSE flywheel velocity in the current Shooter
  subsystem. Automatic stop cancels shooting but does not turn the flywheel off.
  Use manual mode and its off controls to target zero velocity.
- The old unconditional full-power command in Teleop Main was removed so speed
  presets control the flywheel through the Shooter subsystem.
- Button edges and hold timers are updated every loop, including across mode
  changes. Mode changes clear feeding and pending requests. Releasing right
  bumper also cancels a pending automatic shot while the blocker is opening.
- The intake power telemetry uses `intakeMotor`, replacing the sample's missing
  `roller` field. Left bumper also resets the local reverse flag.
- Preset speeds, hood angles, aiming geometry, and physical behavior still need
  validation on the robot. This change does not retune those values.

Intake pivots: idle raises both sides. Intake and feed lower only the selected
side. Reversed intake lowers both sides. Side selection persists until the next
left-trigger press, including while idle; simultaneous presses switch once.
