# PROJECT STATUS

**Project:** Adaptive Path Planning for Autonomous Vehicles on Unstructured Indian Roads (SIH)
**Innovation:** IR-PSC — Indian-Road Predictive Safety Corridor
**Status date:** 2026-09-15 (Phase 2) · branch `phase2-demo`

---

## Phase 2 headline (2026-09-15)

| | |
|---|---|
| **MATLAB execution** | ❌ **Still never executed in MATLAB** — MATLAB is not installed |
| **GNU Octave 11.3.0 execution** | ✅ Whole pipeline, tests and demo executed and debugged in Octave (test-only shims in `tools/octave`) |
| **Octave test suites** | ✅ **117 / 118 pass** (unit 94/95, closed-loop 12/12, behavioural 11/11). The one failure, `testFrenetRoundTrip`, is a pre-existing inconsistency (see Known issues) |
| **Python tests** | ✅ 174 / 174 pass (`existing_work/mathworks_scraper`, Python 3.14.3) |
| **Frenet cross-check** | ✅ 200 / 200 cases agree (CSV produced by `exportFrenetReference` executed in Octave; worst difference 3.6e-15 m) |
| **Primary 3D demo (`runDemo`)** | ✅ `runDemo` itself executed end to end in Octave with the viewer updating inside the loop — goal reached, 0 collisions, identical metrics on repeat (numbers below) |
| **Other five SIH scenarios** | ⚠️ Run without errors, but **do not yet reach their goals and four record contacts** (table below) |
| **Simulink / Stateflow / RoadRunner** | ❌ Not available; no `.slx`, `.sfx`, `.rrscene` exists. SIH RoadRunner requirement **NOT MET** |

> **Every number in this section was measured in GNU Octave, not MATLAB.** Octave random numbers differ from MATLAB's, so MATLAB runs with the same seed will not reproduce these numbers exactly. Re-run in MATLAB (`setupPaths; runAllTests('all'); runDemo`) before quoting any of them.

### Primary demo — `scenarios/demo/scenarioDemo.m`, IR-PSC, seed 1, full simulated sensor chain (Octave)

| Metric | Value |
|---|---|
| Goal reached | **yes**, at t = 76.9 s (limit 90 s) |
| Collisions (static / dynamic contact steps) | **0** (0 / 0) |
| Worst clearance to anything | 0.53 m |
| Average speed | 4.04 m/s |
| Safe-stop episodes | 9 — all at the cow standing in the road and the crossing pedestrian |
| Emergency-braking steps | 0 |
| Planner failure cycles | 8 |
| Potholes | #1 moderate avoided · #2 minor passed without a wheel entry · #3 severe avoided · #4 wide moderate patch (unavoidable) crossed at 3.5–3.9 m/s (cap 4.0) |
| Wall-clock time in Octave | 151 s for 76.9 s simulated |

With perfect perception (diagnostic) an earlier build also reached the goal; the table above is the full-perception run of the final build.

### All scenarios, seed 1 (Octave, full simulated perception)

| Scenario | Planner | Goal | Collisions | Worst clearance | Avg speed | Longest stop | Notes |
|---|---|---|---|---|---|---|---|
| demo | IR-PSC | ✅ 76.9 s | 0 | 0.53 m | 4.04 m/s | — | primary demo |
| demo | baseline | ❌ (90 s) | 2 | −0.53 m | 1.88 m/s | 59.3 s | fixed candidates |
| village | IR-PSC | ❌ (45 s) | 1 | −0.12 m | 1.05 m/s | 27.8 s | |
| urban | IR-PSC | ❌ (40 s) | 1 | −0.20 m | 1.84 m/s | 13.4 s | |
| highway | IR-PSC | ❌ (45 s) | 0 | 2.00 m | 4.90 m/s | 0.1 s | slow behind trucks, no stall |
| market | IR-PSC | ❌ (60 s) | 2 | −1.40 m | 0.21 m/s | 48.8 s | |
| cattle | IR-PSC | ❌ (35 s) | 1 | −0.04 m | 3.92 m/s | 3.9 s | after fixing a follower spawn overlap (was 2 contacts) |

**Contact diagnosis** (first frame of each contact, from the logs):

| Scenario | Ego at contact | Other road user | Who moved into whom |
|---|---|---|---|
| village | stopped (SAFE_STOP) | oncoming motorcycle, 7.0 m/s | scripted actor into stationary ego |
| urban | stopped (SAFE_STOP) | bus, 7.0 m/s | scripted actor into stationary ego |
| market | 2.3 m/s | weaving motorcycle, 5.5 m/s | **ego into actor — a genuine planner failure** |
| market | stopped (SAFE_STOP) | oncoming auto-rickshaw, 3.2 m/s | scripted actor into stationary ego |
| cattle | stopped (SAFE_STOP) | oncoming truck, 10 m/s | scripted actor into stationary ego |
| demo (baseline) | 1.4 m/s | bus, 8 m/s | both moving |
| demo (baseline) | stopped | auto-rickshaw, 6 m/s | scripted actor into stationary ego |

Most contacts are non-reactive scripted actors driving into a vehicle that has already stopped — a limitation of the actor model (they never yield) combined with the planner stopping in a place where an oncoming vehicle needs the space. The planner does not reason about *where* to wait; that is a real gap, not only a scenario artefact. The market contact at speed is a planner failure to be investigated.

**These are honest negative results for five of the six scenarios.** Only the primary demo was tuned end to end in Phase 2. See *Contact diagnosis* below for who hit whom. A single seed is not a statistical result.

### What was verified, and how

- **Vehicle moves under its own control:** ego pose comes only from `bicycleModelStep` driven by `purePursuitControl` + `longitudinalControl` (behavioural tests `testVehicleMovesAndReachesGoalOnClearRoad`, `testPullsAwayFromStandstill`).
- **Trajectory changes with the situation:** `testTrajectoryIsReplannedWhenHazardAppears`; demo log shows avoid / follow / yield / slow behaviours at the scripted events.
- **Potholes detected, confirmed, classified, and handled:** `testPotholeDetectedConfirmedAndClassified`, `testSeverePotholeAvoidedWhenThereIsRoom`, `testUnavoidablePotholeCrossedSlowly`.
- **Static obstacles cannot be driven through unnoticed:** `testRefereeDetectsDrivingIntoStaticObstacle`, `testAvoidsStaticObstacleWithoutContact`, `testStopsBeforeFullyBlockedRoad`.
- **Visualisation matches simulation state:** off-screen snapshots of the recorded demo in chase, overview and top views were inspected against the log (Octave `qt` renderer).

### Known issues (Phase 2)

1. Never executed in MATLAB; MATLAB may raise errors Octave did not.
2. Five of six scenarios do not complete; four record contacts (above). Next step: a "wait where the oncoming road user can pass" behaviour, and investigation of the market motorcycle contact.
3. `testFrenetRoundTrip` fails: `frenetToCartesian` uses smoothed vertex normals and `projectPointOnPath` exact segment normals, so on a coarse, sharply bent 4-point polyline they are not exact inverses. On planner corridors (1 m stations, gentle curvature) the mismatch is centimetres. Pre-existing; left failing rather than weakened.
4. Jerk is a reported comfort criterion, not a rejection reason (see `docs/PHASE2_CHANGES.md` §3).
5. Scripted actors: only `Follower` actors react (gap keeping when behind the ego). No yielding, no negotiation.
6. The drivable-space grid is ground truth handed to the planner; static obstacles are not sensed.
7. No ride dynamics: potholes cannot jolt, damage or destabilise the vehicle; wheel entries are logged with speed.
8. No detector exists for vehicles, animals or potholes; detections are statistical simulations of sensors.
9. Octave runs about 2x slower than real time on the demo; MATLAB speed is unmeasured.

---

## Historical record: status on 2026-09-07 (before Phase 2)

## Headline

| | |
|---|---|
| **Source code** | ✅ Complete — 86 MATLAB files, 19 Python files |
| **Documentation** | ✅ Complete — 20 documents |
| **Python pre-flight** | ✅ **COMPLETE — 174/174 tests pass (executed, measured)** |
| **MATLAB static pre-flight** | ✅ **COMPLETE — 0 findings after one safe fix** |
| **MATLAB runtime validation** | ❌ **NOT YET EXECUTED** |
| **Simulink runtime validation** | ❌ **NOT YET EXECUTED** |
| **Stateflow runtime validation** | ❌ **NOT YET EXECUTED** |
| **RoadRunner runtime validation** | ❌ **NOT YET EXECUTED** |
| **Results / metrics** | ❌ **None exist** |
| **RoadRunner scenes** | ❌ **Not built — SIH requirement NOT MET** |

> **No MATLAB code in this project has ever been executed.** MATLAB, Simulink, Stateflow and RoadRunner are not installed on the authoring machine (`LAPTOP-PPU6OF7R`, verified 2026-09-07 21:53).
>
> Static analysis is clean, but **that is not evidence the code runs.** It checks parse-level structure only. **Expect real failures on first execution** — most likely array-orientation (row vs column) errors.

---

## Pre-flight verification — what has genuinely been done

Full detail: [`docs/PREFLIGHT_REPORT.md`](docs/PREFLIGHT_REPORT.md)

### ✅ Python pre-flight — COMPLETE, EXECUTED

| | |
|---|---|
| Command | `python -m pytest` (from `existing_work/mathworks_scraper`) |
| Result | **174 passed, 0 failed, 0 errors, 0 skipped** |
| Exit code | **0** |
| Runtime | ~1.5 s, fully offline |
| Interpreter | Python 3.14.3, pytest 9.1.1, numpy 2.5.2 |
| Reproduced | Yes — run twice, identical |

Per file: `test_parser` 52 · `test_fetcher` 36 · `test_exporters_and_translate` 32 · **`test_python_port` 31** (Frenet motion model) · `test_transports` 23.

The README's long-standing claim of *"174 tests, fully offline"* is now **verified by measurement** rather than carried as an assertion.

*Not yet verified on Python 3.11/3.12, which `SETUP.md` recommends for the destination machine.*

### ✅ MATLAB static pre-flight — COMPLETE (static text analysis, NOT execution)

Conservative static analysis of all **86 project `.m` files**. The 10 third-party MathWorks reference listings in `existing_work/` were excluded from the verdict.

**Final result: 0 CRITICAL · 0 HIGH · 0 MEDIUM · 0 LOW**

Sixteen checks passed: delimiter balance · `function`/`end` structure · filename/declaration match · chained-index-on-call · undefined helpers · argument-count mismatch · naming · absolute paths · placeholders · missing dependencies · duplicate names · toolbox calls in base-MATLAB files · doc/status consistency · struct-array field order · builtin shadowing · documentation link resolution.

**One genuine defect found and fixed** — see below. A further 35 first-pass findings were traced to bugs in the analyser itself (tokeniser transpose-vs-string, `end`-counting, regex) and were **false positives**; the analyser was corrected, not the project.

### ❌ MATLAB / Simulink / Stateflow / RoadRunner runtime — NOT YET EXECUTED

Blocked. Requires the destination laptop.

---

## Code fix applied during pre-flight

| File | Change | Severity | Risk |
|---|---|---|---|
| `perception/fusion/fuseDetections.m` | Renamed local variable **`all` → `allDets`** at six sites (lines 66, 71, 78, 94, 95, 107), with an explanatory comment | MEDIUM | Minimal |

**Why.** The variable `all` shadowed MATLAB's `all()` builtin for the remainder of the function. The code *worked* — the only genuine `all()` call sits in `combineGroup()`, a separate scope — but it is fragile in a way that fails confusingly: anyone later adding `all(someLogical)` inside `fuseDetections` would silently get struct indexing instead of a logical reduction. MATLAB's own Code Analyzer flags this pattern.

The builtin call at line 143 in `combineGroup()` was correctly left untouched. All static checks were re-run after the change and remain clean.

**This is the only functional source change made since packaging.**

---

## Phase status

| Phase | Status |
|---|---|
| 0 · Environment inspection | ✅ COMPLETE |
| A–O · Source authoring and packaging | ✅ COMPLETE |
| **P1 · Python pre-flight** | ✅ **COMPLETE — 174/174 pass** |
| **P2 · MATLAB static pre-flight** | ✅ **COMPLETE — 0 findings** |
| 1 · Environment verification on destination | ⛔ **BLOCKED — needs MATLAB** |
| 2 · Project initialization + unit tests | ⛔ BLOCKED |
| 3 · Core planner validation | ⛔ BLOCKED |
| 4 · IR-PSC behaviour verification | ⛔ BLOCKED |
| 5 · Baseline comparison | ⛔ BLOCKED |
| 6 · Closed loop | ⛔ BLOCKED |
| 7 · Stateflow | ⛔ BLOCKED — **STATEFLOW = NOT VERIFIED · FALLBACK = USED** |
| 8 · Simulink | ⛔ BLOCKED |
| 9 · RoadRunner scenes | ⛔ BLOCKED |
| 10 · Sensor pipeline | ⛔ BLOCKED |
| 11 · Five-scenario validation | ⛔ BLOCKED |
| 12 · Ablation study | ⛔ BLOCKED |
| 13 · Visual results | ⛔ BLOCKED |
| 14 · Documentation update | ⛔ BLOCKED — nothing new to record until execution |

**Phase 1 remains BLOCKED until MATLAB, Simulink, Stateflow and RoadRunner are available on the destination laptop.**

---

## Software status (authoring machine)

Verified 2026-09-07 21:53 on `LAPTOP-PPU6OF7R` by registry, uninstall database, and recursive executable search across C:, D:, E:.

| Component | Status |
|---|---|
| MATLAB | **NOT INSTALLED** |
| Simulink | **NOT INSTALLED** |
| Stateflow | **NOT INSTALLED** |
| RoadRunner | **NOT INSTALLED** |
| MathWorks toolboxes (all) | **NOT INSTALLED** |
| Python 3.14.3 + pytest + numpy | ✅ working |
| Git 2.53.0 | ✅ working |

---

## What exists

### Source code — complete, statically clean, unexecuted

| Area | Files | Label |
|---|---|---|
| Geometry and data structures (`utils/`) | 14 | REAL |
| Configuration (`config/`) | 3 | REAL |
| IR-PSC planner (`planner/`) | 18 | REAL |
| Prediction (`prediction/`) | 4 | REAL over SIMPLIFIED |
| Perception (`perception/`, `sensors/`) | 4 | SIMPLIFIED / FALLBACK |
| Decision (`decision/`) | 1 | FALLBACK F3 |
| Vehicle (`vehicle/`) | 3 | FALLBACK F7 / SIMPLIFIED F8 |
| Metrics (`metrics/`) | 12 | REAL |
| Scenarios (`scenarios/`) | 8 | SIMPLIFIED |
| Experiments (`experiments/`) | 2 | REAL |
| Scripts (`scripts/`) | 5 | REAL |
| Visualization (`visualization/`) | 2 | REAL |
| Tests (`tests/`) | 6 | REAL — **never run** |
| Simulink builder (`simulink/`) | 1 | NOT VERIFIED, incomplete by design |
| Python (`python/`) | 2 | Supporting |

**Zero placeholder files.**

### Preserved prior work — verified working

`existing_work/mathworks_scraper/` — 35 files, byte-for-byte copy. **Its 174 tests pass.** Includes `python_port/frenet_motion_model.py`, the offline fixture `tests/fixtures/frenet_page.html`, and the 10 extracted MathWorks `.m` listings (third-party reference material, never executed).

---

## What does NOT exist, and why

| Missing | Reason | How to obtain |
|---|---|---|
| `.slx` Simulink model | Binary; only Simulink creates one | `simulink/buildClosedLoopModel.m` (incomplete, never run) |
| `.sfx` Stateflow chart | Binary; only Stateflow creates one | `decision/stateflow/STATEFLOW_SPEC.md` |
| `.rrscene` / `.rrproj` | RoadRunner is a visual 3D editor | `roadrunner/ROADRUNNER_PLAN.md` |
| `.prj` MATLAB Project | MATLAB generates and manages it | `scripts/createProject.m` |
| Any figure, metric or result | Requires an executed run | Run it |

**Nothing above was fabricated.** That is precisely why each is absent.

---

## Outstanding gaps

### 🔴 Critical

1. **No MATLAB code has been executed.** Static analysis is clean but proves only that the text is structurally well-formed.
2. **RoadRunner scenes not built.** SIH requires ≥2 detailed scenes. Depends on licensing, still unconfirmed.

### 🟡 Important

3. No detector trained — auto-rickshaw, pushcart and cattle are not stock detector classes.
4. Simulink model incomplete — builder creates blocks, wiring is manual.
5. MATLAB test results unknown — expect first-run failures.

### 🟢 Acknowledged limitations

6. Scripted actors do not react to the ego vehicle — no negotiation demonstrable.
7. Greedy track association can swap tracks in dense crowds.
8. Kinematic vehicle model — no tyre, suspension or powertrain dynamics.
9. Heuristic weights throughout, not fitted to data.
10. Confidence is a designed heuristic, not a calibrated probability.

---

## Next actions

Follow [`docs/DESTINATION_FIRST_RUN.md`](docs/DESTINATION_FIRST_RUN.md) exactly.

1. Confirm the MathWorks licence, **especially RoadRunner**
2. Install MATLAB (base MATLAB alone runs the whole planner)
3. `setupPaths`
4. `runAllTests('unit')` — **fix failures, record the real numbers below**
5. `runAllTests('all')` — including the reproducibility test
6. `demoScenario` on all five scenarios
7. `runAllExperiments(1:10)`
8. Build the two required RoadRunner scenes
9. Write up, including negative results

---

## Test results

### Python — MEASURED ✅

| Suite | Passed | Failed | Date |
|---|---|---|---|
| `test_parser.py` | 52 | 0 | 2026-09-07 |
| `test_fetcher.py` | 36 | 0 | 2026-09-07 |
| `test_exporters_and_translate.py` | 32 | 0 | 2026-09-07 |
| `test_python_port.py` | 31 | 0 | 2026-09-07 |
| `test_transports.py` | 23 | 0 | 2026-09-07 |
| **Total** | **174** | **0** | **exit code 0** |

### MATLAB — NOT YET RUN

**Leave blank until genuinely executed. Do not pre-fill.**

| Suite | Passed | Failed | Date |
|---|---|---|---|
| `testGeometry` | — | — | not run |
| `testCorridor` | — | — | not run |
| `testPredictionAndRisk` | — | — | not run |
| `testPlannerCore` | — | — | not run |
| `testDecisionAndVehicle` | — | — | not run |
| `testClosedLoop` | — | — | not run |

## Experiment results

**No result is recorded here in advance.**

| Study | Status |
|---|---|
| Baseline vs IR-PSC | not run |
| Ablation (5 variants) | not run |

---

## Claims not made

No safety certification · no guaranteed collision avoidance · no zero-accident claim · no real-world readiness · no real sensor performance · no detector capability · no calibrated probabilities · no real-time-on-target claim · **no MATLAB execution results** · **no simulation results**

---

**Last verified: Python suite executed and passing (174/174). MATLAB static analysis clean. Zero MATLAB execution. Zero simulation results.**
