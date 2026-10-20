# Robot parameters

Every value you may need to set, grouped by the class it lives in. Each class keeps its own
parameters at the top of the file, and each shows up in FTC Dashboard's **Config** panel under
the class name.

**How each value gets set**

- **AUTO**: a tester measures it and prints a line to paste. Nothing to work out by hand.
- **MEASURE**: you get it with a tape measure, a datasheet, or by counting gear teeth.
- **TUNE**: you adjust it by watching the robot. The tester named in the table tells you which
  way to move it.
- **RULE**: fixed by the game manual. Don't change it.
- **LEAVE**: the default is fine for almost every robot.

**Two things to remember**

1. Dashboard edits are lost when the robot restarts. Once a value is final, type it into the
   default in the `.java` file and commit it.
2. Values marked **restart** are read once when the opmode initializes. Changing them in
   Dashboard does nothing until the next init.

---

## Order to tune in

Each step depends on the ones before it.

| # | Do this | Tester (Driver Station, group "test") | Sets |
|---|---|---|---|
| 0 | Measure the robot | (tape measure) | `TRACK_WIDTH`, `WHEEL_BASE`, Pinpoint offsets, gearing |
| 1 | Check odometry | Localization Test (TeleOp) | Nothing. It's a pass/fail check |
| 2 | Drive limits | Drive Characterizer | `V_FORWARD`, `V_STRAFE`, `MAX_WHEEL_SPEED`, `STATIC_POWER`, starting gains |
| 3 | Grip limits | Slip Test (run twice) | `A_FORWARD`, `A_STRAFE` |
| 4 | Path gains | Pathing Tuner, then Gain Sweeper | `CONTOUR_GAIN`, `HEADING_GAIN`, `ACCEL_LEAD`, `SAFETY` |
| 5 | Flywheel | Flywheel Auto Tuner | `kV`, `kA`, `kS`, `FF_TABLE`, `MAX_VELOCITY_ERROR` |
| 6 | Turret | Turret Calibrator | `ZERO_POSITION`, `POSITION_PER_RADIAN` |
| 7 | Intake sensor | Intake Sensor Test (TeleOp) | `PRESENT_CM`, color hue ranges |
| 8 | Shot table | (shoot and record) | `ShotTable2` entries |

---

## DriveConstants: drivetrain and pathing

`subsystems/drive/DriveConstants.java`

### Speed and grip limits (what paths are planned against)

| Variable | What it does | How | Starting value |
|---|---|---|---|
| `V_FORWARD` | Top speed driving straight forward, in/s | **AUTO** Drive Characterizer | 81.175 (measured on the `pedro3` branch) |
| `V_STRAFE` | Top speed strafing sideways, in/s. Always lower than forward | **AUTO** Drive Characterizer | 66.84 (measured on the `pedro3` branch) |
| `A_FORWARD` | Hardest forward acceleration or braking before the wheels slip, in/s² | **AUTO** Slip Test, `STRAFE=false` | 70 (placeholder) |
| `A_STRAFE` | Same, sideways | **AUTO** Slip Test, `STRAFE=true` | 52 (placeholder) |
| `STALL_RATIO` | How much harder the motors push from a standstill than at top speed | **LEAVE** | 3.5 |
| `SAFETY` | Paths are planned at this fraction of the limits above, leaving room for corrections | **TUNE** Pathing Tuner (lower it if it reports saturation) | 0.85 |

### Geometry

| Variable | What it does | How | Starting value |
|---|---|---|---|
| `TRACK_WIDTH` | Left wheel center to right wheel center, in | **MEASURE** | 14.0 |
| `WHEEL_BASE` | Front wheel center to back wheel center, in | **MEASURE** | 13.0 |
| `MAX_WHEEL_SPEED` | One wheel's speed at full power, in/s. Must be at least `V_FORWARD` | **AUTO** Drive Characterizer | 85 |

`Globals.TRACK_WIDTH` (11.27) is a separate older value used by `Pose2d`. Make it match
`TRACK_WIDTH`.

### Path follower

| Variable | What it does | How | Starting value |
|---|---|---|---|
| `CONTOUR_GAIN` | Pulls the robot back onto the line when it drifts sideways | **TUNE** Pathing Tuner / Gain Sweeper | 3.2 |
| `LAG_GAIN` | Speeds the robot up along the path when it falls behind | **TUNE** Pathing Tuner | 0.9 |
| `HEADING_GAIN` | Turns the robot back to the planned heading | **TUNE** Pathing Tuner / Gain Sweeper | 4.0 |
| `ACCEL_LEAD` | Pushes a little harder while accelerating, to cover motor delay. Fixes lag that only appears during acceleration | **TUNE** Pathing Tuner / Gain Sweeper | 0.03 |
| `STATIC_POWER` | Minimum motor power whenever the robot should be moving, to overcome friction. Without it the robot can stop just short of the end of a path | **AUTO** Drive Characterizer | 0.06 |
| `STRAFE_GAIN` | Boosts strafe commands, because mecanum wheels slip sideways | **AUTO** Drive Characterizer (apply it if above 1.15) | 1.25 |
| `FORWARD_GAIN` | Scales forward commands | **LEAVE** at 1.0 | 1.0 |
| `GVF_CONTOUR_GAIN` | How sharply the steering field turns back toward the path | **LEAVE** | 0.9 |
| `GVF_LAG_GAIN` | How strongly the steering field speeds up when behind | **LEAVE** | 0.25 |
| `START_VELOCITY_FLOOR` | Minimum speed for the first 2 in, to get moving | **LEAVE** (raise it if paths stall at the start) | 6.0 |
| `POSITION_TOLERANCE` | How close to the end point counts as arrived, in | **TUNE** to your accuracy needs | 1.0 |
| `PATH_TIMEOUT_FACTOR`, `PATH_TIMEOUT_EXTRA` | If a path runs longer than planned time × factor + extra seconds, the point-to-point PID finishes it | **LEAVE** | 2.0, 1.5 |

### Point-to-point PID (`goToPoint`, not paths)

| Variable | What it does | How |
|---|---|---|
| `xPID`, `yPID` | Forward / sideways position control | **TUNE** by hand |
| `turnPID`, `hPID` | Heading control | **TUNE** by hand |
| `TURN_K_STATIC` | Constant power added to overcome turning friction | **TUNE** |
| `X_THRESH`, `Y_THRESH`, `H_THRESH` | How close counts as arrived | **TUNE** |
| `WAYPOINT_THRESH` | Looser arrival window for points the robot drives through | **TUNE** |

---

## PinpointLocalizer: odometry

`subsystems/drive/localizers/PinpointLocalizer.java`. All position, heading and velocity come
from the goBILDA Pinpoint.

| Variable | What it does | How | Starting value |
|---|---|---|---|
| `X_POD_OFFSET` | Sideways distance from the tracking point to the forward pod, in (left is positive) | **MEASURE** | 3.391 |
| `Y_POD_OFFSET` | Forward distance from the tracking point to the strafe pod, in (forward is positive) | **MEASURE** | 0.582 |
| `POD_TYPE` | Pod model (sets ticks per inch). **restart** | **MEASURE** | goBILDA 4-bar |
| `X_DIRECTION`, `Y_DIRECTION` | Pod directions. **restart** | **TUNE** in Localization Test: pushing forward must increase x, pushing left must increase y | FORWARD, REVERSED |
| `ACCEL_FILTER` | Smoothing on the computed acceleration (used for shoot-on-the-move). 1 = no smoothing | **LEAVE** | 0.3 |

**The tracking point is the turret's center.** The offsets above are measured from there, so
`ROBOT_POSITION` is where the turret is on the field, which is what aiming needs. Re-measure
the offsets if the turret moves.

The Pinpoint is reset when auto or a tester initializes. Teleop keeps whatever position the
Pinpoint already has, so it continues from where auto ended.

---

## Flywheel: shooter speed (LQR)

`subsystems/shooter/Flywheel.java`. Two motors (`shooter1`, `shooter2`) drive one wheel;
`shooter1`'s encoder reads the speed.

### Hardware

| Variable | What it does | How | Starting value |
|---|---|---|---|
| `TICKS_PER_MOTOR_REV` | Encoder ticks per motor revolution | **MEASURE** (motor datasheet; goBILDA = 28) | 28 |
| `WHEEL_REVS_PER_MOTOR_REV` | Wheel turns per motor turn | **MEASURE** (20:30 gearing gives 4000 rpm wheel from a 6000 rpm motor) | 20/30 |
| `WHEEL_DIAMETER` | Flywheel diameter, in. Speeds are surface speed in in/s | **MEASURE** | 3.0 (check this) |
| `MOTOR_REVERSED` | Flip if the wheel spins backwards. **restart** | **TUNE** | false |
| `MOTOR_2_OPPOSED` | The motors face each other, so one runs reversed. Flip if they fight (wheel barely moves, motors get hot). **restart** | **TUNE** | true |

### Model and controller

| Variable | What it does | How | Starting value |
|---|---|---|---|
| `kV` | Volts per in/s of speed. Carries most of the voltage | **AUTO** Flywheel Auto Tuner / Characterizer | 0.0155 (placeholder) |
| `kA` | Volts per in/s² of acceleration | **AUTO** Flywheel Auto Tuner / Characterizer | 0.0012 (placeholder) |
| `kS` | Volts to overcome friction | **AUTO** Flywheel Auto Tuner / Characterizer | 0.35 (placeholder) |
| `FF_TABLE` | Measured volts at each speed. Replaces the straight-line kV/kS estimate, which drifts at high speed | **AUTO** (paste the `FF_TABLE.put` lines into the static block) | empty |
| `USE_FF_TABLE` | Use the table when it has entries | **LEAVE** | true |
| `MAX_VELOCITY_ERROR` | Speed error, in/s, worth spending `MAX_EFFORT` volts to fix. Smaller = stiffer | **AUTO** Flywheel Auto Tuner | 8.0 |
| `MAX_EFFORT` | Volts the feedback is allowed to use | **LEAVE** | 8.0 |
| `CONTROL_DT` | Loop period the controller is designed for, s | **LEAVE** (match your loop time) | 0.02 |

### Behavior

| Variable | What it does | How | Starting value |
|---|---|---|---|
| `AT_VEL_THRESH` | Error, in/s, within which `atVel()` allows a shot | **TUNE** to shot accuracy | 20 |
| `MAX_SPIN_UP_ACCEL` | Limits how fast the target speed ramps up, to stop the belt skipping | **TUNE** (lower it if the belt skips) | 900 |
| `SPIN_UP_THRESH` | The ramp only applies below this speed | **LEAVE** | 150 |
| `VELOCITY_FILTER_LOW` / `_HIGH` / `_THRESH` | Smoothing on the speed reading | **LEAVE** (lower `_LOW` if the auto tuner reports "noisy") | 0.05 / 0.5 / 60 |
| `SHOT_DIP` | Speed drop, in/s, that counts as a shot. Updates the ball count | **TUNE**: watch `Flywheel : Shots` while firing | 25 |
| `SHOT_LOCKOUT_MS` | Minimum time between counted shots | **LEAVE** | 150 |

---

## Turret: three servos, 1:1, with encoder

`subsystems/shooter/Turret.java`

**Range warning:** the servos are 1:1 with the turret, so the turret can only turn as far as a
servo can: about 322° for the configured `AXON_MINI_EXTENDED`. The requested window of −310°
to +50° is 360°, so it doesn't fit. The code automatically narrows the limits to the angles
the servos can actually reach (about −291° to +31° with the current settings). The Turret
Calibrator prints a warning that confirms this. To get the full 360°, the servos need more
travel (reprogram them, or change the gearing).

| Variable | What it does | How | Starting value |
|---|---|---|---|
| `ZERO_POSITION` | Servo position (0 to 1) that points the turret straight ahead | **AUTO** Turret Calibrator | 0.0963 |
| `POSITION_PER_RADIAN` | Change in servo position per radian of turret turn (the sign is the direction) | **AUTO** Turret Calibrator | −1/322° |
| `TICKS_PER_TURRET_REV` | Encoder ticks per turret revolution (8192 for a REV through-bore, times any gearing) | **MEASURE**. Set it **before** running the calibrator | 32768 |
| `ENCODER_REVERSED` | Flip if the angle reads backwards when the turret turns counter-clockwise | **TUNE** by turning it by hand | true |
| `SERVO_TYPE`, `SERVO_REVERSED` | Servo model and per-servo direction. **restart** | **MEASURE** | Axon Mini Extended, all reversed |
| `ENCODER_PORT` | Motor whose encoder port the turret encoder is plugged into. Empty = no encoder. **restart** | **MEASURE** | empty (unset) |
| `MIN_ANGLE`, `MAX_ANGLE` | Mechanical limits | **MEASURE** | −310°, +50° |
| `LIMIT_MARGIN` | Keep-out zone inside each limit | **LEAVE** | 5° |
| `IN_POSITION_TOLERANCE` | Error within which `inPosition()` allows a shot | **TUNE** to shot accuracy | 1.5° |
| `TRIM_GAIN` | How fast the encoder correction learns the servo's offset | **LEAVE** | 2.0 |
| `MAX_TRIM` | Largest correction allowed. Must be more than the backlash the calibrator measures | **TUNE** if the calibrator warns | 4° |
| `TRIM_WINDOW`, `TRIM_MAX_TARGET_RATE` | Trim only learns near the target and while the target is nearly still | **LEAVE** | 6°, 0.5 rad/s |
| `TARGET_RATE_FILTER`, `LEAD_LOOPS`, `MAX_TARGET_RATE` | Leads a moving target to cover servo delay | **LEAVE** | 0.9, 1, 8 |
| `ZERO_SETTLE_MS` | How long the servos must sit at 0 before the encoder is zeroed there | **LEAVE** | 1200 |
| `USE_ENCODER` | Use the encoder for angle, trim and `inPosition()` | **LEAVE** | true |

There's no LQR on the turret. Servos have their own position controller built in.

---

## Intake: left and right, pivot servos, two entrance sensors, flower mechanism

`subsystems/intake/Intake.java`. Two motors (`intake`, `intake2`) run together. The side whose
pivot drops is picked from strafe direction. A flower mechanism (CR servo plus an analog
encoder) handles pollen retrieval from the FLOWER. One entrance sensor per side counts balls.

| Variable | What it does | How | Starting value |
|---|---|---|---|
| `CAPACITY` | Most balls the robot may hold (game rule G407) | **RULE** | 4 |
| `PRELOAD` | Balls held at the start of auto | **RULE** | 4 |
| `PRESENT_CM` | Distance-sensor reading that counts as "ball here" | **TUNE** Intake Sensor Test (use a POLLEN; it reads farther away) | 4.0 |
| `BEAM_ACTIVE_LOW` | For beam-break sensors: the beam reads low when blocked | **TUNE** in Intake Sensor Test | true |
| `MIN_PRESENT_MS`, `MIN_GAP_MS` | Debounce, so a bouncing or wobbling ball counts once | **LEAVE** | 30, 40 |
| `intakePivotUpPosition`, `intakePivotDownPosition` | Pivot servo angles | **TUNE** by eye | 0.0, 1.6 |
| `SIDE_DEADBAND` | Strafe command needed before switching intake side | **LEAVE** | 0.05 |
| `intakeRollerPower` | Intake motors while collecting | **TUNE** | 1.0 |
| `shootRollerPower` | Intake motors while feeding the shooter | **TUNE** | 1.0 |
| `EJECT_POWER`, `REJECT_POWER` | Spitting balls out (manual eject / automatic reject) | **TUNE** | 0.7, 1.0 |
| `REJECT_MIN_MS`, `REJECT_MAX_MS` | How long a 5th ball or opponent NECTAR is reversed out | **TUNE** | 250, 800 |
| `flowerMechDownPosition`, `flowerMechUpPosition` | Flower mechanism targets, rad | **TUNE** | 4.5, 0.0 |
| `flowerMechPID` | Flower mechanism position control | **TUNE** | P = 1 |
| `flowerMechPositionThreshold` | How close counts as arrived, rad | **TUNE** | 0.1 |
| `flowerMechEncoderMaxVoltage` | Encoder voltage for one full turn | **MEASURE** | 3.3 |
| `YELLOW_HUE_*`, `BLUE_HUE_*`, `RED_HUE_*`, `MIN_SATURATION` | Color sorting, so opponent NECTAR is rejected (G408). Only used if a sensor reports color | **TUNE** Intake Sensor Test | see code |
| `LIFT_WHILE_FEEDING` | Lift both pivots while shooting | **LEAVE** | true |

The entrance sensors can be a distance or color sensor (REV Color Sensor V3, 2m distance) or a
digital beam-break. The code detects which. If a sensor isn't configured, counting on that side
is disabled rather than crashing init.

## Shooter

`subsystems/shooter/Shooter.java`

| Variable | What it does | How |
|---|---|---|
| `AUTO_STOP_WHEN_EMPTY` | Stop shooting by itself when the ball count reaches 0 | **LEAVE** true |
| `EMPTY_GRACE_MS` | Keep feeding this long after reaching 0, in case the count was wrong | **LEAVE** 300 |
| `ShotTable2` entries | Distance → flywheel speed and hood angle | **MEASURE** by shooting. Fill it only after the flywheel holds speed consistently |

---

## Hardware names (robot configuration)

The code looks these names up at init. Every one must exist in the active configuration on the
Control Hub, or the opmode crashes before it starts. The two entrance sensors are optional.

| Name | Device |
|---|---|
| `leftFront`, `leftRear`, `rightFront`, `rightRear` | Drive motors |
| `pinpoint` | goBILDA Pinpoint |
| `intake`, `intake2` | Intake motors (run together) |
| `leftIntakePivot`, `rightIntakePivot` | Intake pivot servos |
| `flowerMechanism` | Flower mechanism CR servo |
| `intakeFlowerMechEncoder` | Flower mechanism analog encoder |
| `leftEntranceSensor`, `rightEntranceSensor` | ELC entrance sensors (optional) |
| `shooter1`, `shooter2` | Flywheel motors (`shooter1` needs its encoder plugged in) |
| `turret1`, `turret2`, `turret3` | Turret servos |
| `hood1` | Hood servo |
| `flywheelBlocker` | Gate servo |

The turret encoder is read through a motor's encoder port. Set `Turret.ENCODER_PORT` to that
motor's name. It's empty by default, which turns the encoder off: the turret then runs on the
servos' own position estimate and the Turret Calibrator won't run.
