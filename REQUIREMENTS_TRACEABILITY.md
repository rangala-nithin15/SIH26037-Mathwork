# Requirements Traceability

Each SIH requirement mapped to the code that addresses it, with an honest status.

**Status key:** ✅ implemented (not verified) · ⚠️ partial · ❌ not met · 🔬 simulated only

**Reminder:** ✅ means *the code exists and is complete*. **Nothing has been executed.** No requirement below has been demonstrated working.

---

## Part A — The 18 functional requirements

| # | Requirement | Status | Implementation | Honest note |
|---|---|---|---|---|
| 1 | Adaptive path planning | ✅ | `planner/irpscPlanner.m` | Replans at 5 Hz; corridor adapts continuously to measured free space |
| 2 | Unstructured roads, missing/poor lane markings | ✅ | `corridor/extractCorridor.m` | **No lane detection exists in the pipeline.** Scenarios contain no markings at all |
| 3 | Mixed traffic | ✅ | `config/objectClasses.m`, all scenarios | 10 classes with distinct sizes, speeds and agility |
| 4 | Cars, buses, trucks, auto-rickshaws, two-wheelers, bicycles, pedestrians, pushcarts, animals | ⚠️ | `objectClasses.m` | **Planning and prediction handle all 9. No detector recognises them** — see requirement 12 |
| 5 | Sudden direction changes | ✅ | `irregular_motion/irregularMotionModel.m` | `cutLeft` / `cutRight` hypotheses, weighted by class agility |
| 6 | Informal merging | ✅ | `scenarioUrbanIntersection.m`, `scenarioHighwayMerge.m` | Auto-rickshaw merges without signalling; shallow-angle highway merge |
| 7 | Wrong-way movement | ✅ | `predictConstantVelocity.m` + scenarios | Prediction is direction-agnostic; oncoming actors in village and market share the same space |
| 8 | Unmarked pedestrian crossing | ✅ | `irregularMotionModel.m` `cross` hypothesis | Weighted up when a pedestrian is slow **and near a corridor edge** |
| 9 | Unclear road edges | ✅ | `extractCorridor.m` | Rays that find no edge lower `corridor.quality`, which lowers confidence, which changes behaviour |
| 10 | Potholes / road irregularities | ⚠️ | `buildRoadGrid.m` `extras` | Modelled as **planning obstacles** only. The kinematic vehicle model has no ride response — see below |
| 11 | Multi-sensor perception (camera, LiDAR, radar) | 🔬 | `sensors/`, `perception/` | Three sensors with complementary error models, fused by inverse covariance. **Geometric simulation, not real sensing** |
| 12 | Object detection and tracking | ⚠️🔬 | `simulateDetections.m`, `multiObjectTracker.m` | **Tracking is real (EKF + GNN). Detection is NOT — no detector has been trained.** See `docs/PERCEPTION_NOTES.md` |
| 13 | Short-term motion prediction | ✅ | `prediction/predictObstacles.m` | 3 s horizon, 0.2 s steps, with uncertainty and manoeuvre hypotheses |
| 14 | Collision-risk-aware planning | ✅ | `risk/predictedOccupancyRisk.m`, `lateralRiskGrid.m` | Uncertainty-inflated hazard ellipses; risk grid is **time-aware** |
| 15 | Continuous collision-free trajectory | ⚠️ | `deformTrajectory.m`, `generateTrajectory.m` | Continuous: yes. **"Collision-free" is not claimed** — checks reduce risk, they do not guarantee freedom |
| 16 | Real-time replanning | ⚠️ | `runScenario.m` (`planEvery`) | Replans at 5 Hz in simulation. **"Real-time" is not established** — no target-hardware timing exists |
| 17 | Vehicle control | ✅ | `purePursuitControl.m`, `longitudinalControl.m` | FALLBACK F7 — pure pursuit + PI instead of MPC |
| 18 | Closed-loop simulation | ✅ | `scripts/runScenario.m` | Genuinely closed: dynamics output drives the next sensor frame |

### Notes on the partial items

**#4 and #12 — the detection gap.** Auto-rickshaws, pushcarts and cattle are not classes any stock COCO detector provides. Recognising them needs a custom annotated dataset that does not exist. The planning and prediction layers handle all nine classes properly; the perception layer simulates their detection. This distinction is recorded in code via the `.cocoNative` flag in `objectClasses.m`.

**#10 — potholes.** They can be marked non-drivable in the occupancy grid, and the planner then routes around them with no special-case code — a genuine strength of the drivable-space approach. What must **not** be claimed is ride response: `bicycleModelStep.m` has no suspension, tyre or vertical dynamics. Potholes are planning obstacles, not dynamic disturbances.

**#15 — "collision-free".** The planner performs predictive risk assessment, an independent geometric clearance check, and a feasibility check. Those reduce risk within the modelled world. They cannot guarantee collision freedom, and this project does not claim they do.

**#16 — "real-time".** Replan latency is measured (`metricReplanLatency.m`) and reported with its 95th percentile, because the tail governs deadline behaviour. But these are MATLAB interpreted timings on a development machine. Establishing real-time performance would need generated code on target hardware, which has not been done.

---

## Part B — The five mandatory validation scenarios

| Scenario | Status | Implementation | RoadRunner |
|---|---|---|---|
| A. Unmarked village road | 🔬 | `scenarios/village/scenarioVillage.m` | ❌ **required, not built** |
| B. Busy unsignalized urban intersection | 🔬 | `scenarios/urban_intersection/…` | ❌ **required, not built** |
| C. Highway merge with slow vehicles | 🔬 | `scenarios/highway_merge/…` | not required |
| D. Dense market area | 🔬 | `scenarios/market/…` | not required |
| E. Sudden cattle crossing | 🔬 | `scenarios/cattle_crossing/…` | not required |

All five run today on base MATLAB. All five are **geometric abstractions, not RoadRunner scenes**, and are labelled SIMPLIFIED SIMULATION everywhere they appear.

### ❌ The RoadRunner requirement is NOT MET

> *"At least two detailed RoadRunner scenes, including a village road and an urban intersection."*

**No RoadRunner file exists in this package.** RoadRunner was not installed on the machine where this project was written, and no scene was fabricated.

Full specifications for both required scenes are in [`roadrunner/ROADRUNNER_PLAN.md`](roadrunner/ROADRUNNER_PLAN.md) — dimensions, features, actors, and the MATLAB↔RoadRunner integration steps to verify.

This is the single largest outstanding gap. It depends on RoadRunner being licensed, which is still an open question. If it is not licensed, fallback F9 applies and **the submission must say so explicitly** rather than presenting a substitute as the real thing.

---

## Part C — The 13 IR-PSC hierarchy steps

| # | Step | Status | Implementation |
|---|---|---|---|
| 1 | Estimate drivable space | ✅ | `corridor/seedCenterline.m` |
| 2 | Identify road boundaries | ✅ | `corridor/extractCorridor.m` |
| 3 | Track surrounding road users | ✅ | `perception/tracking/multiObjectTracker.m` |
| 4 | Predict short-term occupancy | ✅ | `prediction/predictObstacles.m` |
| 5 | Represent uncertainty | ✅ | `prediction/uncertainty/predictionUncertainty.m` |
| 6 | Estimate collision/conflict risk | ✅ | `risk/predictedOccupancyRisk.m`, `timeToConflict.m` |
| 7 | Continuous road-aligned preferred trajectory | ✅ | `corridor/extractCenterline.m` |
| 8 | Deform locally around predicted hazards | ✅ | `deformation/deformTrajectory.m` |
| 9 | Maintain boundary and obstacle margins | ✅ | `corridor/corridorBounds.m`, `safety/checkClearance.m` |
| 10 | Check vehicle feasibility | ✅ | `safety/checkFeasibility.m` |
| 11 | Temporally smooth the trajectory | ✅ | `extractCorridor.m`, `deformTrajectory.m`, `generateTrajectory.m` |
| 12 | Confidence-aware behaviour | ✅ | `safety/computeConfidence.m`, `decision/decisionLogic.m` |
| 13 | Reduce autonomy / stop safely | ✅ | `safety/safeStopTrajectory.m`, `decisionLogic.m` |

---

## Part D — Supporting requirements

| Requirement | Status | Implementation |
|---|---|---|
| Stateflow decision logic | ⚠️ FALLBACK F3 | `decisionLogic.m` is complete and working; `STATEFLOW_SPEC.md` specifies the chart. **No `.sfx` exists** |
| Simulink closed-loop model | ⚠️ | `buildClosedLoopModel.m`. **No `.slx` exists**; the builder is incomplete and never executed |
| Baseline comparison | ✅ | `planner/baseline/baselinePlanner.m`, `experiments/runComparison.m` |
| Ablation study | ✅ | `experiments/runAblation.m`, five variants |
| Metrics | ✅ | `metrics/` — all 10 required metrics |
| Reproducibility | ✅ | Single seeded `RandStream` per run; identical seeds across planners, enforced by construction |
| Beginner documentation | ✅ | README, SETUP, RUN_GUIDE, JUDGE_DEMO_GUIDE, plus per-function headers |

---

## Part E — Requirements deliberately NOT claimed

Stated so nobody has to infer them:

| Not claimed | Why |
|---|---|
| Safety certification | None sought, none held |
| Guaranteed collision avoidance | Checks reduce risk; they do not bound real-world behaviour |
| Zero accidents | Zero collisions in a simulation means zero in that simulation |
| Real-world readiness | Simulation research code |
| Real sensor performance | The sensor model is geometric |
| Detector capability | No detector trained or evaluated |
| Real-time on target hardware | No generated code, no target timing |
| Calibrated probabilities | Confidence and hypothesis weights are designed heuristics |
| Interaction / negotiation | Scripted actors do not react to the ego vehicle |

---

## Summary

| | Count |
|---|---|
| Functional requirements fully addressed in code | 13 of 18 |
| Partially addressed, with the gap documented | 5 of 18 |
| Not met | **RoadRunner scenes (2 required)** |
| IR-PSC steps implemented | 13 of 13 |
| Scenarios implemented | 5 of 5 (as MATLAB simulations) |
| **Requirements verified by execution** | **0 — nothing has been run** |
