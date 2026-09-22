# Run Guide

Every command, what it does, and what to expect.

**Before anything:** `setupPaths` — once per MATLAB session.

---

## ⭐ The main demonstration (start here)

```matlab
cd <repository folder>
setupPaths
runDemo
```

That is the whole procedure. A window opens and the **live closed-loop simulation** runs on the primary demo road (`scenarios/demo/scenarioDemo.m`):

| Where | What you see | Where it comes from |
|---|---|---|
| Left, 3D view | Road, houses, trees, parked truck, stalls, potholes (dark depressions), moving road users, the **ego car** (white, cyan roof stripe and sensor pod, cyan ring) | Scenario ground truth; ego pose from the **bicycle model** driven by the **controllers** |
| Cyan ribbon + line | **The trajectory the car is following right now** — amber when following/yielding/blocked, red during a safe stop | `irpscPlanner` output at this step |
| Thin dashed grey-blue line | Preferred path (drivable-corridor centre with keep-left preference) | `plan.preferredPath` |
| Faint cyan line behind the car | Where the car has been in the last 7 s | ego poses |
| Red dashed line | A candidate the planner **rejected** (clearance/feasibility) | `plan.rejectedTraj` |
| Teal/green band | Drivable corridor found by ray-casting free space | `plan.corridor` |
| Yellow → orange → red cells | Predicted-occupancy **risk** the dynamic programme optimised over (low / medium / high) | `plan.riskGrid.R` |
| Violet cells | **Pothole traversal cost** | `plan.riskGrid.potholeCost` |
| Orange-red ellipses | Predicted occupancy of road users at 1 / 2 / 3 s (1-σ) | `plan.preds` |
| Yellow wire boxes + labels | **Tracked** objects (tracker output, may differ from truth) | `multiObjectTracker` |
| Coloured dots | Raw camera / LiDAR / radar detections | `simulateDetections` |
| Rings with labels | Confirmed pothole tracks: severity, depth and planned action | `detectPotholes`, `potholeTracker` |
| Red wall | Planned stop point | trajectory speed reaches 0 |
| Right panel | **State** (CRUISE / SLOW / AVOID / STOP / RESUME), **risk** and **confidence** (LOW / MEDIUM / HIGH with bars), TTC, speed, detections, potholes, **planner** (SAFE / AVOIDING / FOLLOWING / WAITING / STOPPED), pothole tracks, **decision log** | `decisionLogic`, planner, trackers |
| Bottom | Speed, risk, confidence over the last 30 s | simulation log |
| Bottom-left of 3D view | Performance read-out: target FPS, achieved FPS, frame / simulation / render time | measured every frame |

The decision log explains behaviour in steps, for example `[DETECT] Cattle ahead` → `[PREDICT] Path conflict in 2.1 s` → `[PLAN] Holding back for predicted conflict` → `[DECISION] STOP`. Every line restates something the simulation reported (tracker, pothole tracker, planner flags and status, decision-logic state); nothing is inferred beyond that.

Keyboard (click the figure first): **1** chase · **2** overview · **3** top-down · **f** full · **p** planning · **s** sensing · **m** minimal · **space** pause/resume · **q** stop.

**How fast the live view runs.** In a live run the planner, not the renderer, sets the pace (see the measured numbers in `docs/VISUALIZATION.md`). `runDemo` draws every simulation state and keeps the car and camera moving between full redraws, but it does not pretend to a frame rate the simulation cannot supply. For a smooth presentation, record a run and use **replay** (below).

### Options

```matlab
runDemo(struct('camera', 'overview'))                     % start in overview
runDemo(struct('visualMode', 'planning'))                 % full | planning | sensing | minimal
runDemo(struct('videoFile', 'results/irpsc_demo.mp4'))    % record (VideoWriter, base MATLAB)
runDemo(struct('realTime', true))                         % pace display to simulated time
runDemo(struct('scenario', 'village'))                    % any of the six scenarios
runDemo(struct('planner', 'baseline'))                    % the conventional planner, same road
runDemo(struct('usePerfectPerception', true))             % diagnostic: bypass simulated sensors
[log, M] = runDemo(struct('visible', false));             % no window, just run
```

### Replay a recorded run — the presentation mode

```matlab
[log, M] = runDemo(struct('visible', false));           % record once (no window)
save('results/demo_run.mat', 'log', 'M');
replayDemo('results/demo_run.mat')                                   % 30 FPS (default)
replayDemo('results/demo_run.mat', struct('fps', 60))                % 15 / 20 / 30 / 45 / 60
replayDemo('results/demo_run.mat', struct('fromTime', 40, 'camera', 'top'))
replayDemo('results/demo_run.mat', struct('visualMode', 'planning'))
replayDemo('results/demo_run.mat', struct('videoFile', 'results/replay.mp4'))
```

Replay shows exactly what was logged at each step; it does not re-plan. With no planner in the loop it can run at a steady display rate, and it interpolates the car and road users between the 20 Hz recorded states so motion is continuous. Interpolation moves the picture only; no logged value changes. It prints the measured frame rate when it finishes.

### Changing the look: `visualization/viewer3d/vizConfig.m`

One file holds every display setting — FPS target, visual mode, camera, colours, transparency, layer update rates and which layers are shown (HUD, prediction, risk, corridor, trail, performance read-out). It cannot change planning: planner settings live in `config/irpscConfig.m`, which the viewer only reads.

```matlab
vc = vizConfig('planning', struct('displayFps', 60, 'camera', 'overview'));
replayDemo('results/demo_run.mat', struct('viz', vc))
runDemo(struct('viz', vc))
```

### Running without MATLAB (GNU Octave, for development checks)

Octave is **not** the target platform, and results from it must be labelled as Octave results. It is useful for checking the code runs:

```
cd <repository folder>
octave --no-gui
>> addpath('tools/octave'); setupOctave
>> r = runOctaveTests();                  % unit + integration + behaviour suites
>> [log, M] = runScenario('demo', @irpscPlanner, [], struct('seed', 1));
```

`tools/octave` provides a seeded `RandStream` substitute and minimal `functiontests`/`verify*` shims. Random numbers differ from MATLAB's, so an Octave run is not numerically identical to a MATLAB run with the same seed. Octave has no `VideoWriter`; `runDemo` saves PNG frames instead. Octave is roughly an order of magnitude slower than real time on this project.

---

## Quick reference

| Command | What it does | Time |
|---|---|---|
| `setupPaths` | Adds source folders to the path | instant |
| **`runDemo`** | **Main 3D live demonstration** | depends on machine |
| `replayDemo(file)` | Replay a saved run in the 3D viewer | — |
| `runAllTests('unit')` | Unit tests | ~1 min |
| `runAllTests('all')` | Unit + integration | ~5 min |
| `demoScenario('village')` | Animated single run | ~1 min |
| `runScenario('village', @irpscPlanner)` | One run, no animation | seconds |
| `runComparison({'village'}, 1:5)` | Baseline vs IR-PSC | ~2 min |
| `runAblation({'market'}, 1:5)` | Five ablations | ~6 min |
| `runAllExperiments(1:10)` | Everything | ~1 hour |

---

## Testing

```matlab
runAllTests('unit')          % fast, pure numerics
runAllTests('integration')   % full closed-loop runs
runAllTests('all')
```

**Expect failures on first run.** Nothing here has ever been executed.

Run a single suite while fixing something:

```matlab
runtests('testGeometry')
runtests('testCorridor')
runtests('testPredictionAndRisk')
runtests('testPlannerCore')
runtests('testDecisionAndVehicle')
runtests('testClosedLoop')
```

Or a single test:

```matlab
runtests('testGeometry/testCurvatureOfCircle')
```

---

## Running one scenario

```matlab
[log, M] = runScenario('village', @irpscPlanner);
```

Scenario names: `'demo'` · `'village'` · `'urban'` · `'highway'` · `'market'` · `'cattle'` (or a scenario struct)

Planners: `@irpscPlanner` · `@baselinePlanner`

### Options

```matlab
opts = struct( ...
    'seed',                42, ...   % reproducibility — always set this
    'verbose',             true, ... % print progress
    'maxTime',             20.0, ... % cut the run short
    'usePerfectPerception', true);   % bypass sensors and tracking

[log, M] = runScenario('market', @irpscPlanner, [], opts);
```

`usePerfectPerception` is the diagnostic switch worth knowing. It feeds ground truth straight to the planner. If a problem disappears with it on, the problem is in perception, not planning.

### Custom configuration

```matlab
cfg = irpscConfig('village');
cfg.ego.maxSpeed = 8.0;
cfg.deform.maxLateralShift = 1.0;
[log, M] = runScenario('village', @irpscPlanner, cfg, opts);
```

### Reading the result

```matlab
M.collisionCount        % distinct collision events
M.minClearance          % tightest clearance, metres (can be negative)
M.completed             % goal reached AND no collision
M.averageSpeed          % m/s
M.emergencyStops        % SAFE_STOP episodes
M.plannerFailures       % failed planning cycles
M.meanLatency           % seconds per replan
M.p95Latency            % 95th percentile — the tail governs deadlines
M.smoothness            % RMS d(curvature)/ds, lower is smoother
M.predictionError       % RMS prediction error, metres
M.lateralTrackingError  % how well the controller followed the plan
```

**Always read `M.provenance`.** It records that these are simulation results under simplified models, and it travels with the numbers so they cannot be quoted without their caveat.

---

## Demonstrations

```matlab
demoScenario('village')                  % IR-PSC, seed 1
demoScenario('market', 'baseline')       % the conventional planner
demoScenario('cattle', 'irpsc', struct('seed', 7, 'pauseTime', 0.03))
```

Options: `.seed` · `.pauseTime` · `.frameStep` · `.maxTime` · `.record`

### The side-by-side that makes the argument

```matlab
demoScenario('market', 'baseline', struct('seed', 3));
demoScenario('market', 'irpsc',    struct('seed', 3));
```

Same seed means the same world: identical actor timings, identical sensor noise, identical missed detections. Any difference is the planner.

### What to look for in the animation

| Element | Meaning |
|---|---|
| Grey dots | Drivable space. **No lane markings — there are none** |
| Dashed blue | Corridor boundaries, inferred from free space |
| Dotted grey | Corridor centreline: the preferred path |
| **Solid green** | The planned trajectory |
| **Gap between dotted and solid** | **The deformation — the contribution, visible** |
| Solid red | A safe-stop trajectory |
| Orange ellipses | Predicted occupancy; larger = less certain |
| Red boxes | Vulnerable road users |
| Blue boxes | Vehicles |

---

## Experiments

### Baseline comparison

```matlab
r = runComparison({'village','market'}, 1:10, struct('verbose', true));
printComparison(r);
```

Both planners get identical seeds, enforced by construction.

### Ablation study

```matlab
r = runAblation({'market'}, 1:10, struct('verbose', true));
```

Variants: `full` · `no_prediction` · `no_uncertainty` · `no_confidence` · `no_deformation` · `no_smoothing`

A variant showing near-zero deltas means that component is not earning its complexity in that scenario. **Report that honestly.**

### Everything

```matlab
r = runAllExperiments(1:10);
```

Saves to `results/full_evaluation_<timestamp>.mat`.

---

## Tuning

All tunables live in `config/irpscConfig.m`. Nothing else contains a magic constant.

```matlab
cfg = irpscConfig('market');

cfg.corridor.lookaheadDist  = 30;    % see further ahead
cfg.prediction.horizon      = 4.0;   % predict further
cfg.deform.maxLateralShift  = 3.0;   % allow bigger dodges
cfg.safety.lateralClearance = 0.8;   % keep more room
cfg.decision.confLow        = 0.5;   % become conservative sooner
```

### Effects worth knowing

| Change | Effect |
|---|---|
| ↑ `corridor.lookaheadDist` | Earlier reactions, more computation |
| ↑ `prediction.horizon` | Sees hazards sooner, but far-future predictions are vague |
| ↑ `prediction.lateralSpread.*` | Wider hazard ellipses, more cautious, may become timid |
| ↑ `deform.maxLateralShift` | Bigger avoidance manoeuvres, may leave the comfortable centre |
| ↑ `risk.nSigmaOccupancy` | Bigger hazard ellipses, more conservative |
| ↓ `decision.debounceEnter` | Reacts faster, risks chattering |
| ↑ `decision.debounceExit` | Stays cautious longer after a hazard clears |

---

## Debugging

### Inspect one planning cycle

```matlab
setupPaths;
scn = buildScenario('village', 1);
cfg = irpscConfig('village');
ego = makeEgoState(scn.egoStart.pos, scn.egoStart.heading, scn.egoStart.speed);

out = irpscPlanner(scn.grid, ego, [], cfg, []);

out.status              % 'ok' or the failure reason
out.corridor.valid
out.corridor.minWidth
out.corridor.quality
out.confidence
out.confidenceBreakdown % which component dropped it
out.timing              % seconds per stage
```

### Check the corridor alone

```matlab
corr = extractCorridor(scn.grid, ego, cfg, []);
figure; hold on; axis equal;
plot(corr.left(:,1),   corr.left(:,2),   'b--');
plot(corr.right(:,1),  corr.right(:,2),  'b--');
plot(corr.center(:,1), corr.center(:,2), 'k:');
plot(ego.pos(1), ego.pos(2), 'go', 'MarkerSize', 10);
```

If the corridor is wrong, everything downstream is wrong. Check this first.

### Find where a run went bad

```matlab
[log, M] = runScenario('market', @irpscPlanner, [], struct('seed',1));

find(log.collided)                    % which steps collided
unique(log.status)                    % which failures occurred
[~, k] = min(log.minClear);           % tightest moment
log.state{k}                          % what state it was in
log.risk(k)
log.confidence(k)
```

### Common failure statuses

| Status | Meaning | Look at |
|---|---|---|
| `no_corridor` | No drivable space found | The occupancy grid; is the ego inside it? |
| `deformation_infeasible` | Corridor exists, every offset blocked | Genuinely blocked, or margins too strict? |
| `clearance_failed` | Path produced, body would not fit | `cfg.safety` margins, vehicle width |
| `feasibility_failed` | Path produced, vehicle cannot drive it | Curvature limits, speed profile |
| `no_feasible_candidate` | Baseline only — none of its five offsets worked | **This is the fixed-candidate failure mode** |

Since Phase 2 most hazards do **not** produce a failure status: the planner first deforms, slows, follows, yields, or stops before a blockage (see `plan.behaviour.reason`). A failure status means none of those produced a clear, feasible trajectory.

---

## Reproducibility

Every run takes a seed. All randomness flows from one `RandStream`.

```matlab
log1 = runScenario('village', @irpscPlanner, [], struct('seed',42));
log2 = runScenario('village', @irpscPlanner, [], struct('seed',42));
isequal(log1.egoPos, log2.egoPos)     % must be true
```

If that is ever false, something is reading global random state and **every comparison in this project becomes invalid.** `testClosedLoop/testRunIsReproducibleWithSameSeed` guards it.

---

## What you cannot do

- **Get a metric without running something.** `computeMetrics` throws on an empty log, by design.
- **Open a `.slx`, `.sfx`, `.rrscene` or `.prj`.** None exists. Generate them with the builder scripts.
- **Quote real sensor or detector performance.** The sensor model is geometric; no detector has been trained.
- **Claim real-time performance.** Latency numbers are MATLAB interpreted timings on a development machine.
