# Component Register

**Every component in this project carries exactly one of four labels.** This register is the single authoritative list. If a component's label here disagrees with a claim made anywhere else, this file is correct and the other place is a bug.

**Generated:** 2026-09-07 · **Nothing in this project has been executed.** MATLAB, Simulink, Stateflow and RoadRunner are not installed on the machine where it was written.

---

## The four labels

| Label | Meaning |
|---|---|
| **REAL** | A complete, correct implementation of what it claims to be. Not a stub, not a simplification of its own stated purpose. |
| **SIMPLIFIED** | A genuine implementation of a deliberately simplified model. Works, but models less than the real phenomenon. The simplification is stated in the file. |
| **FALLBACK** | A working substitute for an unavailable toolbox component. Real code, but less capable than what it replaces. The thing it replaces is named. |
| **PLACEHOLDER** | Structure only. No working logic. **There are currently zero of these.** |

A fifth status applies orthogonally to the first four:

| **NOT VERIFIED** | The code exists and is complete, but has never been executed in MATLAB. Phase 2: executed in GNU Octave only (see below). |

---

## Verification status — read this first

> **STILL NEVER EXECUTED IN MATLAB.** MATLAB, Simulink, Stateflow and RoadRunner are still not installed on the development machine.
>
> **Phase 2 (2026-09-14/15): the code HAS been executed in GNU Octave 11.3.0**, with the small test-only shims in `tools/octave` (a seeded `RandStream` substitute and `functiontests`/`verify*` stand-ins). That caught and fixed dozens of real defects that static review had not, and produced the measured test results in `PROJECT_STATUS.md`.
>
> Octave is not MATLAB. MATLAB may still raise errors Octave did not (and vice versa), random numbers differ from MATLAB's, and graphics behave differently. **Label every result from this repository as an Octave result until it has been re-run in MATLAB.** The first task on a MATLAB machine is still `setupPaths; runAllTests('all')`, then `runDemo`.

---

## 1. REAL components

Complete implementations. All base-MATLAB only — no toolbox required.

### Core geometry (`utils/`)

| File | What it does |
|---|---|
| `pathArcLength.m` | Cumulative arc length along a polyline |
| `resamplePath.m` | Uniform arc-length resampling, degenerate inputs handled |
| `smoothPath.m` | Moving-average path smoothing with locked endpoints |
| `smoothSeries.m` | Moving-average smoothing of a scalar series |
| `pathHeading.m` | Tangent heading, central differences |
| `pathCurvature.m` | Signed curvature, circumscribed-circle form |
| `wrapToPiLocal.m` | Angle wrapping (local, to avoid a Mapping Toolbox dependency) |
| `projectPointOnPath.m` | Cartesian → road-aligned (s, d) |
| `frenetToCartesian.m` | Road-aligned (s, d) → Cartesian |
| `makeOccupancyGrid.m` | Drivable-space grid structure |
| `isOccupiedAt.m` | Grid query; **out-of-grid counts as occupied** (fail-safe) |
| `rayCastGrid.m` | Ray marching to the first occupied cell |
| `makeObstacle.m` | Canonical road-user record |
| `makeEgoState.m` | Canonical ego state record |

### Configuration (`config/`)

| File | What it does |
|---|---|
| `irpscConfig.m` | Every tunable number, with five scenario profiles and five ablation switches |
| `objectClasses.m` | The ten road-user classes and their physical parameters |
| `vehicleParams.m` | Derived geometry, including the three-disc footprint cover |

### IR-PSC planner (`planner/`)

| File | What it does |
|---|---|
| `irpscPlanner.m` | The 13-step pipeline, top level |
| `corridor/seedCenterline.m` | Greedy maximum-clearance march through free space |
| `corridor/extractCorridor.m` | Boundaries by perpendicular ray casting; **the core of the contribution** |
| `corridor/extractCenterline.m` | Continuous preferred path between boundaries |
| `corridor/corridorBounds.m` | Per-station lateral limits that keep the body on the road |
| `risk/predictedOccupancyRisk.m` | Uncertainty-inflated hazard ellipses |
| `risk/timeToConflict.m` | TTC on the do-nothing trajectory |
| `risk/conflictRisk.m` | Peak risk along a candidate trajectory |
| `risk/lateralRiskGrid.m` | Station × offset risk table, time-aware |
| `deformation/deformTrajectory.m` | Exact dynamic-programming optimum over the lateral grid |
| `trajectory/generateTrajectory.m` | World-frame, time-parameterised output |
| `trajectory/speedProfile.m` | Curvature, risk and acceleration limits with a backward pass |
| `trajectory/scoreTrajectory.m` | Single comparable score, with a progress term |
| `safety/checkClearance.m` | Independent geometric check against grid and obstacles |
| `safety/checkFeasibility.m` | Curvature, lateral acceleration, jerk, steering limits |
| `safety/computeConfidence.m` | Four-part input-quality score |
| `safety/safeStopTrajectory.m` | Controlled stop along the corridor |
| `baseline/baselinePlanner.m` | Fixed five-candidate planner, faithfully implemented |

### Prediction (`prediction/`)

| File | What it does |
|---|---|
| `predictObstacles.m` | Orchestration; folds manoeuvre spread into uncertainty |
| `baseline/predictConstantVelocity.m` | Damped constant-acceleration model with class speed caps |
| `uncertainty/predictionUncertainty.m` | Analytical t⁴ variance growth, class-dependent lateral spread |

### Decision, metrics, experiments, scenarios

| File | What it does |
|---|---|
| `decision/decisionLogic.m` | Six states, asymmetric debounce, emergency override |
| `metrics/computeMetrics.m` + 10 metric files | Every metric, each computed from a log |
| `metrics/aggregateMetrics.m` | Mean, std, min, max, n — spread never omissible |
| `experiments/runComparison.m` | Baseline vs IR-PSC on identical seeds |
| `experiments/runAblation.m` | Five one-component-at-a-time ablations |
| `scenarios/buildRoadGrid.m` | Rasterises a corridor; starts occupied, carves free |
| `scenarios/makeActor.m`, `stepScenario.m`, `buildScenario.m` | Scripted actor framework |
| `scripts/runScenario.m` | The closed loop |
| `scripts/exportFrenetReference.m` | Exports cases for the Python cross-check |
| `visualization/plotScene.m`, `printComparison.m` | Demo figure and results table |
| `python/crosscheck_frenet.py` | Independent Python check of the (s, d) transform |
| All six test files | Real assertions on known-correct cases |

---

## 2. SIMPLIFIED components

Real implementations of deliberately reduced models. Each states its own limits in its header.

| File | Simplified how | What must NOT be claimed |
|---|---|---|
| `perception/detection/simulateDetections.m` | Geometric sensor model over ground truth: FOV, range, occlusion, noise, missed detections, false positives, class confusion. **No image or point cloud is processed.** | Any detector accuracy, precision or recall figure. This is not object detection. |
| `perception/fusion/fuseDetections.m` | Inverse-covariance fusion (correct), but greedy nearest-neighbour association and assumed-independent sensor errors | Optimal association in dense crowds |
| `prediction/irregular_motion/irregularMotionModel.m` | Hand-designed manoeuvre hypotheses and weights | That the weights are calibrated probabilities. They are heuristics. |
| `vehicle/dynamics/bicycleModelStep.m` | Kinematic bicycle, RK2. No tyre slip, no load transfer, no suspension, no powertrain lag | Handling limits, stability, ride quality, or pothole response |
| `scenarios/*/scenario*.m` (all five) | Geometric abstractions: centreline, width profile, blocked regions, scripted actors | **That these are RoadRunner scenes. They are not.** |
| `sensors/sensorConfig.m` | Plausible values, not datasheet values | Real sensor performance |

---

## 3. FALLBACK components

Working substitutes for unavailable toolbox components. Each names what it replaces.

| ID | File | Replaces | Capability lost |
|---|---|---|---|
| **F3** | `decision/decisionLogic.m` | Stateflow chart | Visual state chart, built-in FSM verification. Logic is complete; see `decision/stateflow/STATEFLOW_SPEC.md` |
| **F5** | `perception/tracking/multiObjectTracker.m` | `trackerGNN` / `trackerJPDA` (Sensor Fusion and Tracking Toolbox) | Joint probabilistic association, IMM multi-model. Single CV model, greedy GNN. **Can swap tracks in dense crowds.** |
| **F7** | `vehicle/controller/purePursuitControl.m`, `longitudinalControl.m` | MPC (Model Predictive Control Toolbox) | Constraint handling, preview. Pure pursuit cuts corners on tight bends. |
| **F8** | `vehicle/dynamics/bicycleModelStep.m` | Vehicle Dynamics Blockset | Tyre, suspension, powertrain fidelity (also listed as SIMPLIFIED) |
| **F9** | `scenarios/` (all five) | RoadRunner scenes | Visual fidelity. **The SIH requirement for ≥2 RoadRunner scenes is NOT MET.** See `roadrunner/ROADRUNNER_PLAN.md` |

---

## 4. PLACEHOLDER components

**None.** There are no empty stubs in this project. Every file listed above contains working logic.

---

## 5. NOT VERIFIED — requires MATLAB/Simulink/RoadRunner to run

| File | Requires | Status |
|---|---|---|
| **Every `.m` file in this project** | MATLAB | Never executed in MATLAB (executed in GNU Octave in Phase 2) |
| `simulink/buildClosedLoopModel.m` | MATLAB + Simulink | Never executed. Model is **incomplete** — block wiring must be finished by hand. |
| `scripts/createProject.m` | MATLAB R2019a+ | Never executed |
| `decision/stateflow/STATEFLOW_SPEC.md` | Stateflow | Specification only — **no chart exists** |
| `roadrunner/ROADRUNNER_PLAN.md` | RoadRunner | Specification only — **no scene exists** |
| All test suites | MATLAB | Never run in MATLAB. Run in Octave in Phase 2 — see `PROJECT_STATUS.md` for the measured numbers. |
| `python/crosscheck_frenet.py` | Python + a MATLAB-generated CSV | Phase 2: run against a CSV generated by `exportFrenetReference` **executed in Octave** — 200/200 cases agree |

---

## 5a. Phase 2 components (added 2026-09-14/15)

All base MATLAB. Executed in GNU Octave 11.3.0; not yet in MATLAB.

| File | Label | What it is / what must NOT be claimed |
|---|---|---|
| `planner/IR_PSC/trajectory/speedProfile.m` (rewritten) | REAL | Jerk-limited profile by forward integration |
| `planner/IR_PSC/trajectory/leadVehicleCeiling.m` | REAL | Constant-deceleration gap keeping to a road user in the path |
| `planner/IR_PSC/trajectory/smoothOffsetProfile.m` | REAL | Anchor-preserving smoothing of the DP offset profile |
| `planner/IR_PSC/risk/staticClearanceCost.m` | REAL | Static clearance per (station, offset) from a distance field |
| `planner/IR_PSC/risk/potholeWheelOverlap.m` | SIMPLIFIED | Planar wheel-track vs ellipse geometry; no tyre/suspension model |
| `planner/IR_PSC/risk/potholeCostGrid.m` | REAL (cost model) | Severity-weighted traversal cost; weights are engineering judgement |
| `planner/IR_PSC/risk/isFollowingRoadUser.m` | REAL | Excludes following traffic from braking decisions. **No rear-end reasoning.** |
| `planner/IR_PSC/corridor/truncateCorridor.m` | REAL | Usable-corridor truncation |
| `planner/plannerOutputTemplate.m` | REAL | Shared planner output fields |
| `utils/boxCorners.m`, `boxDistance.m`, `egoFootprint.m` | REAL | Exact rectangle footprints and SAT distance |
| `utils/footprintGridClearance.m`, `gridDistanceField.m` | REAL | Grid clearance, accurate to about one grid cell |
| `utils/rayCastGridMulti.m`, `requireInput.m` | REAL | Vectorised ray casting; cheap input checks (speed only) |
| `perception/detection/detectPotholes.m` | SIMPLIFIED | Statistical camera/LiDAR pothole sensor. **No detector exists; no detection accuracy may be claimed.** |
| `perception/detection/classifyPotholeSeverity.m` | REAL (rule) | Depth thresholds are stated assumptions, not a standard |
| `perception/tracking/potholeTracker.m` | REAL over SIMPLIFIED | Landmark fusion, confirmation, severity from fused depth |
| `perception/tracking/confirmedPotholes.m` | REAL | Filter |
| `scenarios/makePothole.m` | SIMPLIFIED SIMULATION | Ellipse + depth. **No ride/vertical dynamics: a pothole cannot jolt or damage the vehicle.** Wheel entries are logged with speed. |
| `scenarios/roadPath.m` | REAL | Road-following actor waypoints |
| `scenarios/demo/scenarioDemo.m` | SIMPLIFIED SIMULATION | Primary judge demo road. **Not a RoadRunner scene.** |
| `metrics/metricLongestStop.m`, `metricPotholes.m` | REAL | Stall detection; pothole detection/handling summary |
| `visualization/viewer3d/*` | REAL (visualisation) | Schematic 3D in MATLAB graphics. **Not RoadRunner, not Unreal, not photorealistic.** Draws simulation state only. |
| `scripts/runDemo.m`, `scripts/replayDemo.m` | REAL | Live demo entry point; log replay |
| `tests/integration/testBehaviour.m` | REAL | Closed-loop behavioural tests |
| `tools/octave/*` | TEST SHIM | Octave-only compatibility layer. Never on the MATLAB path. |

### Behaviour changes a reader must know about

- **Jerk is a comfort limit.** `checkFeasibility` reports jerk excess in `comfortViolations` and does not reject the plan. `speedProfile` respects the limit by construction. See `docs/PHASE2_CHANGES.md` for why.
- **Scenario geometry changed** in two places so the scenarios are physically passable under the project's own clearance rules: the village truck moved 0.6 m towards the verge (free width 2.3 m → ~2.8 m), and a market pole moved 0.4 m (gap 1.9 m → ~2.4 m). Both remain narrow squeezes.
- **Actors now follow the road** and three opt-in actor behaviours exist (`TriggerS`, `Dwell`, `Follower`). Follower actors keep a gap to the ego; this is the only reaction modelled.

---

## 6. Deliberately absent — and why

These would normally be expected in a project like this. Their absence is a decision, not an oversight.

| Not present | Why |
|---|---|
| `.slx` Simulink model | Binary artefact; only Simulink can create one. Generated by `buildClosedLoopModel.m` on the destination machine. |
| `.sfx` Stateflow chart | Same. Built from `STATEFLOW_SPEC.md`. |
| `.rrscene` / `.rrproj` | Same. RoadRunner is a visual editor. |
| `.prj` MATLAB Project | `.prj` plus a managed `resources/` folder; only MATLAB creates it correctly. Generated by `createProject.m`. `setupPaths.m` is sufficient without it. |
| Any figure, plot or screenshot | Would require an executed run. None has happened. |
| Any metric value, table or result | Same. `computeMetrics.m` **throws** if given an empty log, by design. |
| Trained detector weights | No detector has been trained. |

---

## 7. Toolbox dependency summary

| Component group | Needs | Runs on base MATLAB? |
|---|---|---|
| Geometry, config, planner, prediction, decision, metrics, scenarios, tests | nothing | **Yes** |
| Perception chain (simulated sensors, fusion, tracking) | nothing | **Yes** |
| Controller and dynamics | nothing | **Yes** |
| `buildClosedLoopModel.m` | Simulink | No |
| Stateflow chart (if built) | Stateflow | No |
| RoadRunner scenes (if built) | RoadRunner | No |

**This was a deliberate design decision.** The entire IR-PSC pipeline was kept free of toolbox dependencies so it can be unit-tested and demonstrated on the destination laptop **before** the licensing question is settled. If RoadRunner turns out not to be licensed, the planner still runs, still produces metrics, and still supports the baseline comparison.

---

## 8. Claims this project does NOT make

Stated explicitly so no reader has to infer them:

- **No safety certification.** None sought, none held, none implied.
- **No guaranteed collision avoidance.** The checks reduce risk within the modelled world. They cannot bound behaviour in the real one.
- **No zero-accident claim.** A collision count of zero in simulation means zero collisions in that simulation, nothing more.
- **No real-world readiness.** This is simulation research code.
- **No real sensor performance.** The sensor model is geometric.
- **No detector capability.** No detector has been trained or evaluated.
- **No calibrated probabilities.** Confidence and hypothesis weights are designed heuristics, not validated likelihoods.
- **No measured results of any kind.** Nothing has been run.

---

## 9. What is genuinely claimed as the contribution

The individual techniques — ray casting, road-aligned coordinates, Gaussian uncertainty propagation, dynamic programming, EKF tracking, pure pursuit — are all standard, and **none is claimed as novel**.

The contribution is the **hierarchy**: which quantity is primary, and what the system does when it is unsure.

1. **Drivable space, not lane markings, is the primary planning constraint.** The planner never asks where the lane is.
2. **The preferred path is continuous**, not a choice among fixed lane-offset candidates.
3. **Uncertainty changes behaviour** rather than being merely reported — a vague prediction inflates the hazard ellipse and the planner gives it room, with no special-case code.
4. **Confidence in the system's own inputs is measured and acted upon**, degrading autonomy before a failure rather than after one.
