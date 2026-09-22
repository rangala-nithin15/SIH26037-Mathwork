# Simulink Model Architecture

**Component status:** SPECIFICATION + BUILDER SCRIPT — **no `.slx` file exists**

---

## Why there is no `.slx` in this package

A Simulink model is a binary artefact that only Simulink can create. Simulink was not installed on the machine where this project was written, so no model could be built, opened, or verified.

**No `.slx` was fabricated.** A hand-written file with that extension would not open.

`buildClosedLoopModel.m` generates the model on a machine that has Simulink. That is better than shipping a binary anyway: the model becomes reproducible, reviewable as text, and diffable in git — none of which is true of a committed `.slx`.

> ⚠️ **`buildClosedLoopModel.m` has never been executed.** It is written against the documented Simulink programmatic API and will probably need corrections on first run. It also creates the blocks but does **not** wire them — see [Manual steps](#manual-steps-after-generation).

**The pure-MATLAB pipeline runs the same algorithms today without Simulink.** `scripts/runScenario.m` is the recommended way to generate results while this model is being brought up.

---

## Design decision: Simulink wraps, it does not reimplement

Every stage is a **MATLAB Function block calling the same `.m` functions** the pure-MATLAB pipeline uses.

This is deliberate and it is the most important decision in this file. There is **one** implementation of the planner. A separate Simulink implementation would drift out of step with the MATLAB one, and when they disagreed there would be no way to tell which was authoritative.

**The consequence, stated honestly:** MATLAB Function blocks calling functions that use structs, cell arrays and variable-size data are **not code-generation friendly** without significant rework. This model is for **simulation and demonstration**, not for generating embedded code. Making it codegen-ready is future work and is not a small job.

---

## Model structure

```
┌────────────────────────────────────────────────────────────────────┐
│  sihClosedLoop                                                     │
│  Fixed-step discrete, dt = cfg.sim.dt (0.05 s)                     │
│                                                                    │
│  ┌──────────────┐   groundTruth   ┌──────────────┐                 │
│  │  Scenario    ├────────────────►│   Sensors    │                 │
│  │              │◄──────┐         │              │                 │
│  └──────────────┘  ego  │         └──────┬───────┘                 │
│                         │                │ detections              │
│                         │         ┌──────▼───────┐                 │
│                         │         │  Perception  │                 │
│                         │         │ fuse + track │                 │
│                         │         └──────┬───────┘                 │
│                         │                │ obstacles               │
│                         │         ┌──────▼───────┐                 │
│                         │         │   Planner    │                 │
│                         │         │   IR-PSC     │                 │
│                         │         └──────┬───────┘                 │
│                         │                │ plan                    │
│                         │         ┌──────▼───────┐                 │
│                         │         │   Decision   │                 │
│                         │         └──────┬───────┘                 │
│                         │                │ action                  │
│                         │         ┌──────▼───────┐                 │
│                         │         │  Controller  │                 │
│                         │         └──────┬───────┘                 │
│                         │                │ steer, accel            │
│                         │         ┌──────▼───────┐                 │
│                         └─────────┤   Vehicle    │                 │
│                            ego    │   Dynamics   │                 │
│                                   └──────┬───────┘                 │
│                                          │ ego → Sensors           │
└────────────────────────────────────────────────────────────────────┘
```

## Solver settings

| Setting | Value | Why |
|---|---|---|
| Solver type | Fixed-step | The planner runs at a fixed rate |
| Solver | `FixedStepDiscrete` | No continuous states in the model |
| Fixed step | `cfg.sim.dt` = 0.05 s | 20 Hz |
| Stop time | `cfg.sim.maxTime` | Per-scenario |

A variable-step solver would be both meaningless and slower here.

---

## Block wrapper bodies

Paste these into the corresponding MATLAB Function blocks. Each is a thin wrapper around the shared implementation.

### Scenario

```matlab
function [gtPos, gtVel, gtClass, nActors] = scenarioStep(egoPos, egoHeading, t)
%#codegen
% Advances the scripted actors. Persistent state holds the actor array,
% because Simulink has no other place to keep it between steps.
persistent actors dt initialised
if isempty(initialised)
    scn = buildScenario('village', 1);
    actors = scn.actors;
    cfg = irpscConfig(scn.profile);
    dt = cfg.sim.dt;
    initialised = true;
end
[actors, gt] = stepScenario(actors, t, dt);
% Flatten to fixed-size arrays: Simulink signals cannot be struct arrays.
nActors = numel(gt);
gtPos = zeros(20,2); gtVel = zeros(20,2); gtClass = zeros(20,1);
for i = 1:min(nActors, 20)
    gtPos(i,:) = gt(i).pos;
    gtVel(i,:) = gt(i).vel;
    gtClass(i) = classNameToId(gt(i).class);
end
end
```

### Planner

```matlab
function [trajPos, trajSpeed, risk, confidence, ttc, statusCode] = ...
         plannerStep(egoPos, egoHeading, egoSpeed, obsPos, obsVel, obsClass, nObs)
%#codegen
persistent plannerState grid cfg initialised
if isempty(initialised)
    scn = buildScenario('village', 1);
    grid = scn.grid;
    cfg  = irpscConfig(scn.profile);
    plannerState = [];
    initialised = true;
end
ego = makeEgoState(egoPos, egoHeading, egoSpeed);
obstacles = rebuildObstacles(obsPos, obsVel, obsClass, nObs);

out = irpscPlanner(grid, ego, obstacles, cfg, plannerState);
plannerState = out.state;

trajPos    = padTo(out.traj.pos,   cfg.traj.numPoints, 2);
trajSpeed  = padTo(out.traj.speed, cfg.traj.numPoints, 1);
risk       = out.risk;
confidence = out.confidence;
ttc        = min(out.ttc, 1e6);          % Inf is not a valid Simulink signal
statusCode = statusToCode(out.status);
end
```

### Decision

```matlab
function [stateId, speedLimit, useSafeStop] = ...
         decisionStep(risk, confidence, ttc, statusCode, feasible, clearanceOk)
%#codegen
persistent dstate cfg initialised
if isempty(initialised)
    cfg = irpscConfig('village');
    dstate = [];
    initialised = true;
end
plan = struct('risk', risk, 'confidence', confidence, 'ttc', ttc, ...
              'isSafeStop', false, 'feasible', feasible, ...
              'clearanceOk', clearanceOk, 'status', codeToStatus(statusCode));
[action, dstate] = decisionLogic(plan, cfg, dstate);
stateId     = stateNameToId(action.state);
speedLimit  = action.speedLimit;
useSafeStop = action.useSafeStop;
end
```

### Controller and Vehicle

```matlab
function [steer, accel] = controlStep(trajPos, trajSpeed, egoPos, egoHeading, ...
                                      egoSpeed, speedLimit)
%#codegen
persistent cstate cfg vp initialised
if isempty(initialised)
    cfg = irpscConfig('village');
    vp  = vehicleParams(cfg);
    cstate = [];
    initialised = true;
end
traj = rebuildTraj(trajPos, trajSpeed);
ego  = makeEgoState(egoPos, egoHeading, egoSpeed);
steer = purePursuitControl(traj, ego, cfg, vp);
[accel, cstate] = longitudinalControl(traj, ego, speedLimit, cfg, cstate);
end
```

```matlab
function [egoPos, egoHeading, egoSpeed] = vehicleStep(steer, accel)
%#codegen
persistent ego cfg initialised
if isempty(initialised)
    scn = buildScenario('village', 1);
    cfg = irpscConfig(scn.profile);
    ego = makeEgoState(scn.egoStart.pos, scn.egoStart.heading, scn.egoStart.speed);
    initialised = true;
end
ego = bicycleModelStep(ego, steer, accel, cfg);
egoPos = ego.pos; egoHeading = ego.heading; egoSpeed = ego.speed;
end
```

---

## The signal-flattening problem

Simulink signals must be fixed-size numeric arrays. The MATLAB pipeline passes struct arrays freely. Every block boundary therefore needs flattening and rebuilding, which is where most of the bring-up work will be.

| MATLAB | Simulink signal |
|---|---|
| `obstacles` (1×M struct array) | `obsPos` 20×2, `obsVel` 20×2, `obsClass` 20×1, `nObs` scalar |
| `traj` struct | `trajPos` N×2, `trajSpeed` N×1 |
| `plan.status` char | `statusCode` int8 |
| `action.state` char | `stateId` int8 |
| `Inf` (e.g. TTC) | clamped to 1e6 |

**Write the helper functions once** — `classNameToId`, `statusToCode`, `codeToStatus`, `stateNameToId`, `rebuildObstacles`, `rebuildTraj`, `padTo` — and put them in `simulink/helpers/`. They are small, but getting them wrong produces confusing failures deep in the model.

The 20-obstacle cap is arbitrary. Raise it if a scenario needs more; the cost is signal width, not computation.

---

## Manual steps after generation

`buildClosedLoopModel.m` creates the blocks and sets the solver. It does **not**:

1. **Set the MATLAB Function block code.** Paste the wrappers above into each block.
2. **Connect the blocks.** Port names depend on the function signatures, which only exist after step 1.
3. **Set the initial conditions** in the Scenario and Vehicle blocks.
4. **Break the algebraic loop.** The ego state feeds back into Scenario and Sensors, which is a direct feedthrough loop. Insert a **Unit Delay** on the ego feedback path. Without it Simulink will report an algebraic loop error.

Step 4 is the one most likely to be missed and hardest to diagnose from the error message.

---

## Verifying the model against the MATLAB pipeline

Once it runs, the model and `runScenario.m` must produce the **same trajectory** for the same scenario and seed. They call the same functions, so any divergence is a wiring or flattening bug, not an algorithmic one.

```matlab
log = runScenario('village', @irpscPlanner, [], struct('seed',1));
% then run the model with the same scenario and seed, and compare ego paths
```

**If they disagree, the model is wrong** — the MATLAB pipeline is the reference, because it is the one with a test suite.

---

## What this model is for, and what it is not

**For:** demonstrating the architecture visually to judges, scope-based signal inspection, and future hardware-in-the-loop work.

**Not for:** generating embedded code (struct and cell usage would need substantial rework), or as the primary results generator — `runScenario.m` is faster, tested, and needs no toolbox.

---

**Status: SPECIFICATION + UNVERIFIED BUILDER. No Simulink model has been created, opened, executed or verified by this project.**
