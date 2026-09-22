# System Architecture

How data flows through SIH_Indian_AV, and why it is shaped this way.

---

## 1. The closed loop

```
┌──────────────────────────────────────────────────────────────────────┐
│                                                                      │
│   ┌────────────────────┐                                             │
│   │ SCENARIO           │  scripted actors advance along waypoints     │
│   │ stepScenario.m     │  SIMPLIFIED                                  │
│   └─────────┬──────────┘                                             │
│             │ ground truth (obstacle structs)                        │
│   ┌─────────▼──────────┐                                             │
│   │ SENSORS            │  camera + LiDAR + radar, geometric model     │
│   │ simulateDetections │  FOV · range · occlusion · noise · misses    │
│   │ SIMPLIFIED         │  false positives · class confusion           │
│   └─────────┬──────────┘                                             │
│             │ detections (3 independent lists)                       │
│   ┌─────────▼──────────┐                                             │
│   │ FUSION             │  inverse-covariance weighting                │
│   │ fuseDetections.m   │  LiDAR dominates position, camera the class  │
│   │ SIMPLIFIED         │  radar the velocity                          │
│   └─────────┬──────────┘                                             │
│             │ fused detections                                       │
│   ┌─────────▼──────────┐                                             │
│   │ TRACKING           │  constant-velocity KF + GNN association      │
│   │ multiObjectTracker │  M-of-N confirmation, coasting, deletion     │
│   │ FALLBACK (F5)      │  emits CONFIRMED tracks only                 │
│   └─────────┬──────────┘                                             │
│             │ obstacles (tracked road users)                         │
│   ┌─────────▼──────────────────────────────────────────┐             │
│   │ IR-PSC PLANNER                                     │             │
│   │ irpscPlanner.m                    REAL             │             │
│   │  ┌──────────────────────────────────────────────┐  │             │
│   │  │ 1-2  corridor from drivable space            │  │             │
│   │  │ 3-5  predict road users + uncertainty        │  │             │
│   │  │ 6    predicted-occupancy risk grid           │  │             │
│   │  │ 7-8  continuous path, deformed (DP)          │  │             │
│   │  │ 9-10 margins, spatial + temporal smoothing   │  │             │
│   │  │ 11   feasibility                             │  │             │
│   │  │ 12   confidence                              │  │             │
│   │  │ 13   safe stop if none of the above holds    │  │             │
│   │  └──────────────────────────────────────────────┘  │             │
│   └─────────┬──────────────────────────────────────────┘             │
│             │ plan {traj, risk, confidence, ttc, status}             │
│   ┌─────────▼──────────┐                                             │
│   │ DECISION LOGIC     │  6 states, asymmetric debounce               │
│   │ decisionLogic.m    │  emergency bypasses debounce                 │
│   │ FALLBACK (F3)      │  → speed limit + safe-stop command           │
│   └─────────┬──────────┘                                             │
│             │ action {state, speedLimit, useSafeStop}                │
│   ┌─────────▼──────────┐                                             │
│   │ CONTROLLER         │  pure pursuit (lateral)                      │
│   │ purePursuit +      │  PI with anti-windup (longitudinal)          │
│   │ longitudinalControl│  FALLBACK (F7)                               │
│   └─────────┬──────────┘                                             │
│             │ steer, accel                                           │
│   ┌─────────▼──────────┐                                             │
│   │ VEHICLE DYNAMICS   │  kinematic bicycle, RK2 midpoint             │
│   │ bicycleModelStep.m │  steering-rate limited                       │
│   │ SIMPLIFIED (F8)    │                                              │
│   └─────────┬──────────┘                                             │
│             │ new ego state                                          │
└─────────────┴────────────────────────────────────────────────────────┘
              └──────────────► feeds back to SCENARIO and SENSORS
```

**The loop is genuinely closed.** The ego state produced by the dynamics model determines what the sensors see next. Nothing is fed back from ground truth to the planner — ground truth is used only by the collision referee, which is deliberately independent.

---

## 2. Data structures — the contracts between stages

Four structures carry everything. Each is created by exactly one constructor, so a stage can never receive a subtly different shape depending on who called it.

### `grid` — drivable space (`makeOccupancyGrid.m`)

```
.occ         nRows × nCols logical, true = OCCUPIED
.resolution  metres per cell
.origin      world coordinates of cell (1,1) centre
```

**Out-of-grid queries return occupied.** Space we know nothing about is not space we drive into. This makes the planner fail safe at the edge of sensor coverage.

**There is no lane field.** There is nowhere to put one. A lane-based planner cannot be built on this interface without changing it, which is the point.

### `ego` — vehicle state (`makeEgoState.m`)

```
.pos      [x y] REAR AXLE, not centroid
.heading  yaw, rad
.speed    m/s
.yawRate .accel .steer .time
```

Rear axle throughout: the bicycle model and pure pursuit are both formulated about it, and the footprint discs are offset from it. Mixing reference points would introduce a systematic offset that would look like a controller tuning problem.

### `obstacle` — a road user (`makeObstacle.m`)

```
.id .class .pos .vel .accel .heading .speed
.length .width .confidence .age .posCov
.vulnerable .agility            ← copied from the class table
```

Scenario ground truth, tracker output and hand-written test fixtures all emit this same struct. The planner never knows which produced it.

### `traj` — a trajectory (`generateTrajectory.m`)

```
.pos .heading .curvature .speed .s .times .valid
```

`.times` is arrival time from the planning instant, which is what makes time-aware risk possible.

---

## 3. Where each stage's numbers come from

| Stage | Every constant lives in |
|---|---|
| Corridor | `cfg.corridor` |
| Prediction | `cfg.prediction` |
| Risk | `cfg.risk` |
| Deformation | `cfg.deform` |
| Safety | `cfg.safety` |
| Trajectory | `cfg.traj` |
| Decision | `cfg.decision` |
| Control | `cfg.control` |
| Timing | `cfg.sim` |
| Sensors | `sensorConfig.m` |

**No magic constants outside these.** This is what makes ablation possible: an ablation is a modified copy of one struct, not a code change.

---

## 4. Timing

| Rate | What runs |
|---|---|
| 20 Hz (`cfg.sim.dt = 0.05`) | Sensors, tracking, decision, control, dynamics |
| 5 Hz (`cfg.sim.planEvery = 4`) | The planner |

The planner runs at a quarter rate because corridor extraction and the DP deformation are the expensive parts, and replanning at 5 Hz is ample when the trajectory extends several seconds ahead. Between replans the controller keeps following the last trajectory.

**The exception:** when the decision logic commands a safe stop, a fresh stop trajectory is generated on that step regardless of the replan schedule. An emergency must never wait for the next planning slot.

---

## 5. Why the interfaces are shaped this way

**Occupancy grid, not road network.** A road-network interface would carry lanes, and a planner built on it would inevitably come to depend on them. The grid interface makes "no lane markings required" true at the boundary, not just in the algorithm — and it means a RoadRunner scene, a `drivingScenario` and a hand-built MATLAB scenario are all interchangeable inputs.

**Planners are drop-in interchangeable.** `irpscPlanner` and `baselinePlanner` take identical arguments and return identical fields (enforced by a test). The simulation loop does not know which it is running. Without this the comparison would be measuring two different harnesses.

**One implementation, wrapped by Simulink.** The Simulink model calls the same `.m` functions. A separate Simulink implementation would drift out of step and there would be no way to tell which was authoritative.

**The collision referee uses ground truth.** `checkCollisionAgainstTruth` in `runScenario.m` never looks at the planner's beliefs. A perception failure therefore shows up as a collision, rather than being hidden by the same error that caused it.

**Planner state is passed in and out, not stored.** Temporal smoothing needs memory across cycles, but `persistent` variables would make the planner untestable and would leak between runs, silently breaking reproducibility. State goes in as an argument and comes back as a field.

---

## 5a. Phase 2 additions to the loop

```
scenario actors move (road-following paths, ego-progress triggers, follower gap keeping)
  -> camera / LiDAR / radar  ->  fusion  ->  tracking (radar Doppler as a proper KF update)
  -> camera + LiDAR POTHOLE detection  ->  pothole landmark tracker (confirm, fuse, classify)
  -> IR-PSC planner
       corridor (narrow / blocked stations, spatially aligned temporal blending)
       prediction (class-dependent heading uncertainty)
       risk grid + POTHOLE wheel-track cost + STATIC clearance cost (distance field)
       dynamic programme (speed-scaled smoothness, keep-left preference cue)
       speed ceilings: narrow passage, pothole, lead road user, blockage, end of corridor
       jerk-limited speed profile
       time-aware clearance check  ->  yield (plan a stop before the conflict)
       feasibility check           ->  retry at reduced speed
       safe stop only if nothing above yields a clear, feasible trajectory
  -> decision logic (emergency judged on the PLANNED trajectory; emergencyBrake flag)
  -> pure pursuit + PI with feed-forward (emergency deceleration only when flagged)
  -> kinematic bicycle
  -> referee: exact rectangles vs road users AND the occupancy grid; pothole wheel entries
  -> onStep callback -> live 3D viewer (visualization/viewer3d)
```

**Response cascade.** The planner answers a hazard with the least intrusive response that yields a clear and feasible trajectory: deform → slow → follow → yield → stop before a blockage → safe stop. Emergency braking (`cfg.ego.emergencyDecel`) is used only when the required deceleration exceeds service braking or the conflict on the planned trajectory is imminent (`ttcPlanned ≤ cfg.risk.ttcCritical`).

**Following traffic.** Road users behind the ego travelling the same way are excluded from braking and stopping decisions (they stay in the lateral risk grid). Braking because a follower is predicted to reach you makes that conflict worse; see `isFollowingRoadUser.m`.

**Potholes are not obstacles.** They never enter the occupancy grid. They add a wheel-track cost to the dynamic programme (a pothole between the wheels costs nothing) and a speed cap where a tyre will cross one. Their cost is kept separate from collision risk, so a pothole changes path and speed but never, by itself, triggers an emergency.

**Visualisation is downstream only.** `runScenario` calls `opts.onStep(frame)` after every step. The viewer reads the frame; nothing flows back into the simulation.

---

## 6. Simulink and Stateflow

**No `.slx` or `.sfx` file exists in this package.** Both are binary artefacts only their tools can create, and neither tool was installed on the machine where this was written.

- `simulink/buildClosedLoopModel.m` generates the model — **never executed, incomplete, block wiring must be finished by hand.**
- `decision/stateflow/STATEFLOW_SPEC.md` fully specifies the chart. `decisionLogic.m` is a complete working implementation of that specification and is what the project uses today.

See [`simulink/SIMULINK_ARCHITECTURE.md`](simulink/SIMULINK_ARCHITECTURE.md).

---

## 7. Toolbox dependencies

| Layer | Needs | Runs on base MATLAB? |
|---|---|---|
| Everything in the loop above | nothing | **Yes** |
| `buildClosedLoopModel.m` | Simulink | No |
| Stateflow chart, if built | Stateflow | No |
| RoadRunner scenes, if built | RoadRunner | No |

The entire pipeline is toolbox-free by design, so it can be tested and demonstrated **before** the licensing question is settled.
