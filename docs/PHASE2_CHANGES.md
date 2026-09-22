# Phase 2 — what changed, why, and how it was verified

**Dates:** 2026-09-14 to 2026-09-15 · **Branch:** `phase2-demo`

**Execution environment:** GNU Octave 11.3.0 on Windows 11 (MATLAB is still not installed). Every "verified" below means *executed in Octave*, not in MATLAB. See `tools/octave/README.md` and `PROJECT_STATUS.md`.

The rule followed throughout: **preserve the IR-PSC architecture, fix what is demonstrably wrong, and never relax a safety threshold to make a run pass.** Each fix below names the evidence that forced it.

---

## 1. Why the original code could not drive

A numerical reproduction of the audit, then the first Octave runs, showed the vehicle stopping (and then never moving again) on an *empty straight road*. The causes, in the order they were found:

| # | Defect | Evidence | Fix |
|---|---|---|---|
| 1 | Speed profile had step changes in acceleration; `checkFeasibility` measured jerk 6–33 m/s³ vs a 3 m/s³ limit and rejected every plan | Python port of the original functions; first Octave run | `speedProfile.m` rewritten as a jerk-limited forward integration from the vehicle's current speed and acceleration |
| 2 | At standstill the controller read the target speed exactly where the profile starts (0), so it never pulled away | Octave run: v = 0, a = −0.3 forever | `longitudinalControl.m`: minimum preview distance + feed-forward of the profile slope |
| 3 | A single station narrower than `minWidth` anywhere in the lookahead invalidated the whole corridor → emergency stop tens of metres before a pinch | Audit width scan; Octave | `extractCorridor.m`: NARROW (drive slowly) vs BLOCKED (body does not fit) stations; corridor truncated at the first blocked station and a controlled stop planned before it |
| 4 | Village scenario pinch left 2.3 m free — impassable under the project's own clearance rule | Audit width scan | Scenario geometry: truck moved 0.6 m to the verge (≈2.8 m free, still a narrow squeeze) |
| 5 | Clearance check compared *current* obstacle positions with *every future* trajectory point, and used a 5.6 m circle for a bus | Reading + Octave runs | `checkClearance.m`: time-aware (predicted pose at the sample's time), exact rectangles, reachable samples only |
| 6 | Planner had no "follow" behaviour: slow vehicle ahead ⇒ swerve or emergency stop | Design review | `leadVehicleCeiling.m` |
| 7 | TTC extrapolated the ego in a straight line (off the road on bends) | Reading | `timeToConflict.m`: nominal TTC follows the road; planned TTC follows the trajectory; decision logic uses planned TTC for emergencies |
| 8 | Configured emergency deceleration was never used (controller clamped to service braking) | Reading | `longitudinalControl.m` `emergencyBrake` argument, set by the decision logic |
| 9 | Straight-line actor paths left curved roads by up to 5 m | Audit Python check | `roadPath.m`; all curved-road scenarios updated |
| 10 | Collision referee ignored static obstacles | Reading | `runScenario.m` referee checks exact rectangle vs road users **and** the grid |
| 11 | Radar radial velocity blended 30 %/frame into the tracker's velocity dragged crossing objects to "stationary" | Reading | `multiObjectTracker.m`: Kalman update with the radial-velocity measurement model |
| 12 | Prediction-error metric compared tracker ids with actor ids | Reading | `truthId` carried through obstacles and predictions; metric matches on it |
| 13 | Temporal corridor blending was index-by-index while the car moved | Reading | Spatially aligned blending (closest point on the previous centreline) |
| 14 | Tuning constants hard-coded inside planner functions | Reading | Moved to `irpscConfig.m` (`corridor`, `deform`, `risk`, `traj`, `score`, `confidence`, `baseline`, `fusion`, `tracking`, `pothole`) |

## 2. Defects found only by executing in Octave

| Defect | Evidence | Fix |
|---|---|---|
| Smoothing moved the first offset away from the vehicle; snapping the path start back to the car created a 0.13 1/m kink (6 m/s² lateral) | First plan of the demo rejected | `smoothOffsetProfile.m` (anchor-preserving raised-cosine ramp); station 1 set to the exact lateral offset |
| End-locked path smoothing bunched points (first segment 4.2 m) | Debug print of the trajectory | Resample after smoothing; minimum 0.8 m sample spacing |
| Static obstacles only seen through perpendicular corridor rays: a stall corner between rays was invisible to the DP; the exact check then failed and the car stopped for good | Stuck at s = 145 m, clearance 0.02 m | `gridDistanceField.m` + `staticClearanceCost.m` add static clearance to the DP (one grid cell stricter than the final check) |
| The vehicle braked for a **follower** motorcycle predicted to reach it from behind — self-reinforcing emergency stops | Log analysis: worst risk contributor = motorcycle behind the ego in most emergencies | `isFollowingRoadUser.m`: following traffic excluded from braking decisions, kept in the lateral risk grid |
| Constant 0.15 rad heading uncertainty gave an oncoming car 2.7 m lateral σ after 2 s — risk saturated, passing impossible | Risk = 1.0 while passing a car in its own lane | Class-dependent heading std (vehicles 0.03–0.08 rad, people/animals 0.25–0.30) |
| Seed march could turn at a 1.4 m radius (e.g. at a dead end) | Safe stop near the end of a straight test road | Seed heading change per step bounded by the vehicle's curvature limit |
| The DP's short pothole dodge was smoothed back over the pothole | `potholeCost` columns vs smoothed offsets printed | Surface/static costs dilated along the road by the smoothing reach; finer lateral grid (0.15 m); 0.2 m pothole planning margin |
| A corridor ending at a wall was treated as open road; the car kept 11 m/s | Behavioural test: contact with a road block | Always keep a stop ceiling at the end of the usable corridor ("never outrun the known drivable space") |
| One pothole became two tracks (camera noise vs a fixed 1.5 m gate) | Behavioural test | Gate scaled with detection noise; duplicate tracks merged |
| Finite-difference jerk on floored arrival times near standstill produced large spurious values | Log analysis | Distance-domain acceleration `(v2²−v1²)/2ds` and speed-weighted jerk |

## 3. A deliberate reclassification (read this)

**Jerk is now a comfort criterion, reported but not a rejection reason.** In closed loop the remaining jerk excesses occurred almost exclusively while the vehicle was already braking (an initial condition no plan can undo) or were finite-difference artefacts. Rejecting such a plan replaced it with a safe-stop trajectory that brakes *harder* — the opposite of the intent. The profile generator still respects `cfg.ego.maxJerk` by construction and a unit test enforces that on nominal profiles; jerk excess is returned in `details.comfortViolations`. **Curvature, steering, lateral and longitudinal acceleration remain hard limits, and all clearance limits are unchanged.**

## 4. New capabilities

- **Potholes** as first-class objects: ground truth (`makePothole`), simulated camera + LiDAR detection (`detectPotholes`), landmark tracking with confirmation and severity estimation (`potholeTracker`), wheel-track DP cost (`potholeCostGrid`), speed caps where a tyre crosses one, planner actions `avoid` / `slow` / `straddle`, logged wheel entries with speed, metrics (`metricPotholes`). **No ride dynamics are modelled.**
- **Response cascade** in `irpscPlanner.m`: deform → slow → follow → yield → stop before blockage → safe stop; emergency braking only when needed.
- **Keep-left preference** (`cfg.deform.preferredOffset`) as an optional traffic-side cue on two-way roads.
- **Primary demo scenario** `scenarios/demo/scenarioDemo.m` with ego-progress-triggered events.
- **Live 3D viewer** (`visualization/viewer3d`), `runDemo`, `replayDemo`, video recording through `VideoWriter` (PNG frames where it is unavailable).
- **Behavioural tests** (`tests/integration/testBehaviour.m`) and an **Octave compatibility layer** (`tools/octave`).
- **Performance:** vectorised ray casting, projection, curvature and smoothing; `validateattributes` removed from inner-loop utilities (profiling showed it dominating run time). Results unchanged (unit tests, Frenet cross-check).

## 5. Tests changed, and why

Every changed assertion had never been executed before Phase 2.

| Test | Change | Reason |
|---|---|---|
| `testCorridorBoundsCollapseWhenTooNarrow` | "zero room" = zero-width band + corridor blocked/invalid | Old bounds were forced to contain d = 0 even when that left the road |
| `testSpeedProfileRespectsLateralAccelLimit` | Start below the curve limit; new test for entering a bend too fast | Old profile assumed the car could shed 16 m/s instantly |
| `testAgileClassSpreadsFasterLaterally` | Same speed for both; compare before the σ cap | Test compared speeds, not classes; both values hit the cap at 3 s |
| `testPurePursuitSaturates` | Densely sampled path | Two-point path put the target 30 m away; could never saturate |
| `testDeformationIsSmooth` | Deterministic pseudo-random risk | Unseeded `rand` made the test flaky |
| New: `testSpeedProfileStartsAtCurrentSpeedAndBrakesLegally`, `testSpeedProfileRespectsJerkOnStraightRoad`, `testNarrowButPassableGapIsNotBlocked`, whole `testBehaviour` suite | — | Regression and behaviour coverage |

## 6. Known limitations that remain

See `PROJECT_STATUS.md` — including: never executed in MATLAB; no RoadRunner scenes; no trained detector; kinematic vehicle; scripted actors (followers only keep a gap); no rear-end reasoning; drivable-space grid is ground truth given to the planner, not sensed; heuristic weights; `testFrenetRoundTrip` fails (pre-existing inconsistency between smoothed-normal `frenetToCartesian` and exact-segment `projectPointOnPath` on sharply bent coarse polylines).
