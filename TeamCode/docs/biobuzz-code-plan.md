# BIOBUZZ (2026-27) — Code Plan

Source of truth: BIOBUZZ Competition Manual V1 (Kickoff 2026-09-12). Section
numbers below refer to that manual.

## 1. What the game actually is

**Match:** 30s AUTO → 8s transition → 2:00 TELEOP. FLOWER ownership unlocks at
1:00 remaining.

**Scoring elements (§9.8):** 40 POLLEN (yellow, **2.8 in**) + 8 red / 8 blue
NECTAR (**3.6 in**). Two diameters, two ownership classes. Preload is 4 POLLEN
per robot (§10.3.1).

**Field elements:**
- **HIVE** (§9.6) — center of field, one frame holding a red and a blue hive.
  Each hive is a **bi-stable pivot** with 2 CELLS 18.8 in apart, pivot axis
  43.95 in above the tiles. Cell opening 20 W × 14 H × 12 D in. Launch into the
  upward-facing cell; enough elements flips it → **HIVE TIP**. The target
  physically relocates every tip.
- **FLOWER** (§9.7) — 4 of them on the perimeter wall. 4 in opening, 21.5 in
  above tiles, placed from the top only. POLLEN retrievable from a 3.55 in
  opening at the bottom. NECTAR out only from the top (G418).
- **GARDEN** (§9.3) — 23 × 2 in corner strip, alliance-specific.
- **LOADING ZONE** (§9.3) — 23 × 11 in, park target and the only legal human
  NECTAR entry point (G427).

**Point values (§10.5.5):**

| Action | AUTO | TELEOP |
|---|---|---|
| LEAVE | 3 | — |
| PARK (loading zone) | 5 | 5 |
| HIVE TIP | 20 | 20 |
| Element remaining in upward CELL | — | 2 |
| Bottom NECTAR bonus (per flower) | — | 5 |
| Element in an **owned** FLOWER | — | 2 |
| Element in GARDEN | — | 1 |

RP: SWARM (≥16 combined leave+park), POLLINATOR 1 (≥4 tips), POLLINATOR 2
(≥7 tips), WIN 3 / TIE 1.

**Rules that are really code requirements:**
- **G410** — no NECTAR into a FLOWER before 1:00 remaining. MAJOR FOUL *per
  nectar*. This must be a hard software interlock, not a driver habit.
- **G426** — humans may enter one NECTAR per hive tip, all remaining at 1:00.
  So tip count gates how much nectar exists on the field.
- **G417** — the only legal way to move the hive is launching into the upward
  cell. Do not aim at a tipping hive.
- **G418** — flowers: in from the top, POLLEN out from the bottom only.

## 2. Strategic read (drives what we build first)

Tips are the game. 7 tips = 140 pts + 2 RP, and each tip also releases a
NECTAR. Flowers are an endgame multiplier: owning a flower pays 2 pts for
*every* element in it including the 4 preloaded pollen and the opponent's.
Garden is 1 pt cleanup. LEAVE+PARK is 13 pts/robot — SWARM RP is nearly free
and should never be missed.

Priority: **launcher > flower deposit > garden**. That happens to be the same
priority as last season's robot, which is why most of this repo survives.

## 3. What carries over unchanged

Essentially the whole substrate. Do not rewrite these:

- `subsystems/drive/**` — GVF pathing, Bezier/spline geometry, min-curvature
  optimizer, velocity profiling, friction ellipse, slip detection.
- `subsystems/drive/localizers/**` — Pinpoint, EKF, `nMergeLocalizer`.
- `control/**` — LQR, motion profiles, matrix math, state prediction.
- `utils/**` and `utils/priority/**` — the hardware queue and priority
  device scheduling are season-agnostic and expensive to rebuild.
- `photon/**`, `Robot.java` loop structure, telemetry/datalogging.
- `opmodes/tuning/**` — drive characterization, gain sweep, follow metrics.

## 4. What must be written

### Phase 0 — Repo reset (do first)
The working tree currently has every real opmode staged for deletion and only
`ExampleAuto` + tuning opmodes left. Land that cleanup as one commit, tag the
DECODE final as `decode-final` so last season's autos stay recoverable, then
branch `biobuzz`.

### Phase 1 — Field model (`field/BiobuzzField.java`, new package)
Everything downstream needs one authoritative geometry source. Currently field
constants are scattered in `Globals` (`redTag`, `blueTag`, `tagHeight`).

Write:
- Field-frame poses for both HIVE pivots, and for each CELL in **both** hive
  states (4 cell poses per alliance × 2 states).
- The 4 FLOWER positions + the 21.5 in rim height and 4 in aperture.
- GARDEN and LOADING ZONE polygons, per alliance (needed for park and for
  scoring-aware auto endings).
- AprilTag map: **IDs 30-45**, four clusters of four. 30-33 red far-side cell,
  34-37 red audience-side, 38-41 blue audience-side, 42-45 blue far-side
  (§9.9). Tags are 3.25 in, 36h11, on the **bottom face of each cell facing
  down**, bottom edge toward field center.
- `alliance` mirroring helper so autos are written once.

### Phase 2 — Vision rework (`vision/Vision.java`, `vision/HiveTracker.java`)
`Vision.java` is hardcoded to `AprilTagGameDatabase.getDecodeTagLibrary()` and
the old two-tag layout. Changes:

1. Swap in the BIOBUZZ tag library (or build a custom `AprilTagLibrary` from
   the Phase 1 map if the SDK library lags).
2. **New: `HiveTracker`.** Cluster detections by ID group → identify which
   cell is upward-facing for each alliance, from tag pose/orientation. Expose
   `hiveState(alliance)`, `tipCount`, and a debounced `isTipping` flag so the
   shooter holds fire during a flip (G417).
3. Tag-to-robot-pose feed into `nMergeLocalizer` needs new tag geometry — the
   tags are now on a *moving* structure, so a tag pose is only a valid
   localization fix once the hive state is known and settled. Gate the vision
   correction on `!isTipping`.
4. Cluster-of-4 averaging: four coplanar tags per cell is a much better pose
   solve than one tag. Fuse them rather than taking the first detection.

This is the single largest new-code item and the biggest risk. Start it now.

### Phase 3 — Shooter retarget (`subsystems/shooter/**`)
`Shooter.java` already has the hard part: ballistic solve, hood/turret/flywheel
coordination, shoot-on-the-move compensation. What changes:

- **Target selection is now dynamic.** `ballTarget` must come from
  `HiveTracker` (which cell is up), not a constant. Add a target-provider
  indirection so the aim solver is fed a live `Vector3`.
- **Two projectiles.** POLLEN (2.8 in) and NECTAR (3.6 in) have different mass,
  drag, and compression through the launcher. `ShotTable2` needs to become
  per-element — either two tables or a table keyed on element type. Do not
  assume one interpolation covers both.
- **New aim geometry.** The DECODE goal was a wall-mounted static target; the
  cell is a 20 × 14 in mouth ~44 in up, angled, in the field center, reachable
  from a full 360°. Re-derive the hood range and re-tune `minV0factor*`.
  Expect a genuinely different hood sweep requirement.
- **Fire control:** hold fire while `isTipping`; count launches into the
  current cell and predict the tip so the feeder pauses cleanly.

### Phase 4 — Intake + sorting (`subsystems/intake/Intake.java`)
New requirement with no DECODE analogue: the robot handles **three classes** of
ball (own nectar, opponent nectar, pollen) at **two diameters**.

- Element-type sensing at intake — color sensor for red/blue/yellow, plus a
  size discriminant (breakbeam spacing or the color sensor's proximity return)
  since 2.8 vs 3.6 in is a large mechanical difference.
- A `MagazineState` model: what is held, in what order, of what type. The
  shooter's ballistics and the flower logic both query it.
- Reject path for opponent nectar, or a deliberate decision to launch it into
  our own cell (legal, still 2 pts at rest, and it denies them).
- **Pollen retrieval from the flower base** (3.55 in tall opening) is a
  distinct intake mode with a tight height window — treat it as its own state.

### Phase 5 — Flower deposit (`subsystems/flower/FlowerDeposit.java`, new)
`subsystems/slides/Slides.java` and `park/Park.java` exist and are the natural
starting points. Requirements:
- Place into a 4 in aperture at 21.5 in — a positional, vision-or-odometry
  aligned action, not a launch.
- **Hard G410 interlock:** a `MatchClock` singleton fed from opmode start; the
  deposit subsystem refuses NECTAR before 1:00 remaining and telemeters why.
- Ordering logic: first nectar in at the 1:00 bell captures the **bottom
  bonus** (5), the last nectar in captures **ownership** (2/element). Encode
  both as explicit driver-assist macros.

### Phase 6 — Teleop (`opmodes/Teleop.java`, rewrite)
- Match-phase state machine driven by `MatchClock`: PRE / EARLY (launch only) /
  FLOWER_WINDOW (last 60s) / ENDGAME (last 20s — park).
- Driver-facing indication of hive state and tip count (LED via `LEDWrapper` +
  telemetry), because the human player's nectar entry is gated on tips.
- Auto-aim toggle reusing the existing turret tracking; add a "target the other
  cell" override for when the hive flips mid-cycle.

### Phase 7 — Autonomous
30s, preloaded with 4 POLLEN. Realistic AUTO: LEAVE (3) + tip once or twice
(20-40) + PARK (5). Write:
- `BiobuzzAutoBase` — alliance-mirrored, using `PathBuilder` + GVF as before.
- A close-launch auto that shoots the 4 preloads at the upward cell from a
  fixed, tuned spot, then parks. Highest points per unit of risk.
- A garden/flower-pollen cycling auto once the intake is proven.
- Keep `PathingTuner`, `GainSweeper`, `FollowMetrics` in the loop — they still
  work and the drivetrain retune is cheap.

### Phase 8 — Optional: scoring sim
`src/test/java/.../sim/` already has `SimChassis` and `SimOdometry`. A cheap
match-score model (tips vs. flower ownership vs. garden) would settle strategy
arguments before hardware exists. Low priority, high leverage if the design
team is undecided.

## 5. Suggested order

1. Phase 0 repo reset + `biobuzz` branch.
2. Phase 1 field model (blocks everything).
3. Phase 2 vision / HiveTracker (longest pole, most unknowns).
4. Phase 3 shooter retarget (reuses the hardest existing math).
5. Phase 4 intake sorting (blocked on mechanical design).
6. Phase 5 flower deposit + G410 interlock.
7. Phase 6 teleop, Phase 7 autos.

Phases 1-3 are pure software and can start before the robot exists. Do them now.

## 6. Open questions to resolve on hardware

- **How many elements tip a hive?** The manual never states it (§10.5.1 defines
  the tip mechanically, not by count). Measure it — the whole fire-control and
  RP model depends on this number.
- Does one launcher geometry handle both 2.8 and 3.6 in balls, or do we need
  two paths? This is the biggest architecture fork in the shooter code.
- Is the cell mouth reachable from the loading zone, or must we drive in? Sets
  whether autos are stationary-launch or cycling.
