# SIH_Indian_AV

**Adaptive Path Planning for Autonomous Vehicles on Unstructured Indian Roads**

Core innovation: **IR-PSC — Indian-Road Predictive Safety Corridor**

> *"Lane is an optional cue; drivable space is the primary planning constraint."*

---

## ⚠️ Read this before anything else

**This project has still never been run in MATLAB.** MATLAB, Simulink, Stateflow and RoadRunner are not installed on the development machine.

**Phase 2 (Sept 2026): the code has been executed and debugged in GNU Octave 11.3.0** (a free MATLAB-compatible interpreter), using the small test-only shims in `tools/octave`. The unit, integration and behavioural tests and the full demo scenario run there; the measured results are in [`PROJECT_STATUS.md`](PROJECT_STATUS.md) and every change is explained in [`docs/PHASE2_CHANGES.md`](docs/PHASE2_CHANGES.md). **Octave is not MATLAB** — report these as Octave results, and re-run in MATLAB before quoting anything.

There are also things this package deliberately does **not** contain, because they cannot be created honestly without the software:

| Not present | Why | How to get it |
|---|---|---|
| `.slx` Simulink model | Only Simulink can create one | `simulink/buildClosedLoopModel.m` |
| `.sfx` Stateflow chart | Only Stateflow can create one | `decision/stateflow/STATEFLOW_SPEC.md` |
| `.rrscene` RoadRunner scenes | RoadRunner is a visual 3D editor | `roadrunner/ROADRUNNER_PLAN.md` |
| `.prj` MATLAB Project | MATLAB generates and manages it | `scripts/createProject.m` |
| Any result, figure or metric | Would require an executed run | Run it yourself |

**The SIH requirement for at least two detailed RoadRunner scenes is currently NOT MET.** See [`roadrunner/ROADRUNNER_PLAN.md`](roadrunner/ROADRUNNER_PLAN.md).

Full labelling of every component: [`docs/COMPONENT_REGISTER.md`](docs/COMPONENT_REGISTER.md).

---

## Quick start

```matlab
cd <this folder>
setupPaths                    % adds all source folders to the path
runDemo                       % the live 3D IR-PSC demonstration (primary demo)
runAllTests('all')            % unit + integration + behavioural tests
```

`runDemo` runs the full closed loop — simulated sensors, fusion, tracking, pothole detection, prediction, the IR-PSC planner, decision logic, controllers and vehicle model — and draws every step in a 3D view as it happens. See [`RUN_GUIDE.md`](RUN_GUIDE.md) for options (camera views, video recording, replay).

Then, once the tests pass:

```matlab
runAllExperiments(1:10)       % baseline comparison + ablation study
```

**The whole planner runs on base MATLAB.** No toolbox is required for any of the above. That was a deliberate design decision — see [Design decisions](#design-decisions-worth-knowing).

---

## What this system does

It plans a path for a vehicle on roads that have no usable lane markings, sharing space with cars, buses, trucks, auto-rickshaws, two-wheelers, bicycles, pedestrians, pushcarts and animals that do not follow lane discipline and may not follow the rules at all.

The pipeline, and where each stage lives:

```
scenario ground truth
    → camera + LiDAR + radar simulation        sensors/, perception/detection/
    → multi-sensor fusion                      perception/fusion/
    → multi-object tracking                    perception/tracking/
    → short-term prediction with uncertainty   prediction/
    → IR-PSC planner                           planner/IR_PSC/
    → decision logic (6 states, debounced)     decision/
    → controller                               vehicle/controller/
    → vehicle dynamics                         vehicle/dynamics/
    → back to the sensors (closed loop)
```

## The IR-PSC hierarchy

Thirteen steps, and the file that implements each:

| # | Step | Implementation |
|---|---|---|
| 1 | Estimate drivable space | `seedCenterline.m` |
| 2 | Identify road boundaries | `extractCorridor.m` |
| 3 | Track surrounding road users | `multiObjectTracker.m` |
| 4 | Predict short-term occupancy | `predictObstacles.m` |
| 5 | Represent uncertainty | `predictionUncertainty.m` |
| 6 | Estimate collision risk | `lateralRiskGrid.m`, `conflictRisk.m` |
| 7 | Continuous preferred trajectory | `extractCenterline.m` |
| 8 | Deform locally around hazards | `deformTrajectory.m` |
| 9 | Preserve boundary and obstacle margins | `corridorBounds.m`, `checkClearance.m` |
| 10 | Smooth spatially and temporally | `generateTrajectory.m`, `smoothPath.m` |
| 11 | Check vehicle feasibility | `checkFeasibility.m` |
| 12 | Confidence-aware behaviour | `computeConfidence.m` |
| 13 | Degrade or stop safely | `safeStopTrajectory.m`, `decisionLogic.m` |

Detail: [`INNOVATION_IR_PSC.md`](INNOVATION_IR_PSC.md).

---

## Repository layout

```
SIH_Indian_AV/
├── setupPaths.m               ← run this first
├── config/                    every tunable number, 5 profiles, 5 ablation switches
├── utils/                     geometry, occupancy grid, data structures
├── planner/
│   ├── irpscPlanner.m         the 13-step pipeline
│   ├── IR_PSC/
│   │   ├── corridor/          drivable-space corridor extraction  ← the core
│   │   ├── risk/              predicted-occupancy risk, TTC
│   │   ├── deformation/       dynamic-programming trajectory deformation
│   │   ├── safety/            clearance, feasibility, confidence, safe stop
│   │   └── trajectory/        generation, speed profile, scoring
│   └── baseline/              fixed-candidate planner, for comparison
├── prediction/                constant-velocity + uncertainty + irregular motion
├── perception/                detection simulation, fusion, tracking
├── sensors/                   camera / LiDAR / radar configuration
├── decision/                  6-state logic + Stateflow specification
├── vehicle/                   controller and dynamics
├── scenarios/                 the five SIH scenarios
├── metrics/                   10 metrics + aggregation
├── experiments/               baseline comparison, ablation study
├── visualization/             demo figure, results table
├── tests/                     unit + integration suites
├── scripts/                   runScenario, demoScenario, runAllExperiments
├── simulink/                  model builder + architecture spec (no .slx)
├── roadrunner/                scene specifications (no scene files)
├── python/                    cross-check tool (supporting, not the deliverable)
├── existing_work/             preserved prior Python work
├── docs/                      component register, perception notes
└── results/                   generated output — empty, and must stay empty
```

---

## The five SIH scenarios

| Scenario | Command | What it is built to test |
|---|---|---|
| **Primary demo** | `runDemo` | Every behaviour on one road: potholes (avoid / straddle / slow), following, market corridor shift, cattle, pedestrian |
| A. Unmarked village road | `demoScenario('village')` | The central claim: **no lane markings exist at all** |
| B. Unsignalized intersection | `demoScenario('urban')` | Prediction under occluded crossing traffic |
| C. Highway merge | `demoScenario('highway')` | Early decisions at speed; shallow-angle merge |
| D. Dense market | `demoScenario('market')` | Everything at once; confidence-aware slowdown |
| E. Sudden cattle crossing | `demoScenario('cattle')` | Emergency escalation and safe stop |

---

## Design decisions worth knowing

**The planner depends on base MATLAB only.** Not one line of the IR-PSC pipeline calls a toolbox function. This means the whole thing can be unit-tested and demonstrated **before** the RoadRunner licensing question is settled. If RoadRunner turns out to be unavailable, the planner still runs and still produces the baseline comparison.

**The planner's input is an occupancy grid, not a road network.** That is what lets it work identically on a RoadRunner scene, a `drivingScenario`, or a hand-built MATLAB scenario — and it is what makes "no lane markings required" true at the interface, not just in the algorithm.

**Simulink wraps the MATLAB functions; it does not reimplement them.** There is one implementation of the planner. A second one inside Simulink would silently drift out of step with the first.

**Every experiment is seeded.** Both planners face byte-identical noise, missed detections and false positives. Without that the comparison would measure luck.

**`computeMetrics` throws on an empty log.** By construction, you cannot get a metric out of this project without having run something.

---

## Honest limitations

- Nothing has been executed. No result exists.
- The sensor model is geometric. **No detector has been trained.** Auto-rickshaws, pushcarts and cattle are not classes any stock detector provides — see [`docs/PERCEPTION_NOTES.md`](docs/PERCEPTION_NOTES.md).
- The vehicle model is kinematic: no tyre slip, no suspension, no powertrain lag.
- Scenario actors are **scripted and do not react to the ego vehicle**. These scenarios cannot demonstrate negotiation or mutual yielding.
- The tracker uses greedy association and can swap tracks in dense crowds.
- Confidence is a designed heuristic, not a calibrated probability.
- Potholes are detected (simulated sensors), tracked and planned around or crossed slowly; **ride response is not modelled** — the kinematic vehicle has no suspension.
- The drivable-space grid is a ground-truth map given to the planner; static obstacles in it are not sensed.
- Scripted actors only keep a gap when following the ego; they never yield or negotiate.
- The 3D view is schematic MATLAB graphics, not RoadRunner or a game engine.

## Claims not made

No safety certification. No guaranteed collision avoidance. No zero-accident claim. No real-world readiness. No real sensor performance. This is simulation research code.

---

## Documentation

| Document | For |
|---|---|
| **[`docs/DESTINATION_FIRST_RUN.md`](docs/DESTINATION_FIRST_RUN.md)** | **⭐ Start here on the destination laptop — exact commands, expected output, PASS/FAIL criteria** |
| [`docs/PREFLIGHT_REPORT.md`](docs/PREFLIGHT_REPORT.md) | What has been verified (Python 174/174) and what has not |
| [`SETUP.md`](SETUP.md) | Getting it running on a new machine |
| [`RUN_GUIDE.md`](RUN_GUIDE.md) | Every command, what it does, what to expect |
| [`SYSTEM_ARCHITECTURE.md`](SYSTEM_ARCHITECTURE.md) | How data flows, and why it is shaped this way |
| [`INNOVATION_IR_PSC.md`](INNOVATION_IR_PSC.md) | The contribution, and what is *not* claimed |
| [`REQUIREMENTS_TRACEABILITY.md`](REQUIREMENTS_TRACEABILITY.md) | Each SIH requirement → the code that addresses it |
| [`VALIDATION_PLAN.md`](VALIDATION_PLAN.md) | How this gets validated, and what would falsify it |
| [`JUDGE_DEMO_GUIDE.md`](JUDGE_DEMO_GUIDE.md) | Running the demo and answering hard questions |
| [`PROJECT_STATUS.md`](PROJECT_STATUS.md) | What is done, what is not |
| [`docs/COMPONENT_REGISTER.md`](docs/COMPONENT_REGISTER.md) | **Every component's REAL/SIMPLIFIED/FALLBACK label** |
| [`docs/PERCEPTION_NOTES.md`](docs/PERCEPTION_NOTES.md) | What perception does and does not do |
