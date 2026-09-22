# Destination Laptop — First Run

**Follow this in order. Do not skip ahead.**

Every command is given exactly as you should type it. Where I can tell you what output to expect, I have. Where I cannot, I say so rather than guessing.

---

## ⚠️ Read this first

**No MATLAB code in this project has ever been executed.** It was written on a machine with no MATLAB installed.

**You will hit errors. That is expected, not a sign something is wrong.** Several thousand lines of MATLAB written without an interpreter available will contain mistakes. Finding and fixing them is your first job, and it is normal engineering work.

**Do not hide, skip, or paper over a failure.** A documented failure is useful. A hidden one becomes a question you cannot answer in front of judges. If a test fails, record the exact error and send it back.

What *has* been verified: the Python suite (174/174 passing, measured) and static structural analysis of all 86 MATLAB files (0 findings). Static analysis proves the text is well-formed. **It does not prove the code runs.**

---

## STEP A · Install MATLAB 👤 HUMAN GUI ACTION

**Before installing — confirm your licence.** 👤

1. Sign in at **mathworks.com → My Account → My Software**
2. Write down: licence type, the exact product list, and **whether RoadRunner is included**

RoadRunner is often licensed separately, and SIH requires two detailed scenes. Ask your SIH coordinator and faculty mentor before buying anything.

**Then install:** 👤

Download the installer from mathworks.com → Downloads, choose the newest release your licence allows, install to a drive with plenty of free space, and activate.

> 💡 **The entire IR-PSC planner runs on base MATLAB.** You can install MATLAB alone and immediately do Steps D–G. Toolboxes only unlock Simulink, Stateflow and RoadRunner work.

**Time:** 30–90 minutes.

---

## STEP B · Install MathWorks products 👤 HUMAN GUI ACTION

Tick these in the installer, limited to what your licence permits:

| Priority | Product | Unlocks |
|---|---|---|
| **Essential** | MATLAB | Everything in Steps D–G |
| High | Simulink | Step H — the closed-loop model |
| High | Stateflow | Step H — the decision chart |
| Medium | Automated Driving, Navigation, Sensor Fusion and Tracking | Upgrades to fallback components |
| Medium | Computer Vision, Deep Learning, Lidar | Real perception instead of simulated |
| Low | Model Predictive Control | Upgrade from pure pursuit |
| Low | Vehicle Dynamics Blockset | Upgrade from the bicycle model |

If disk or seats are limited, install **Essential + High** first. Every lower item has a documented fallback already implemented.

---

## STEP C · Install RoadRunner 👤 HUMAN GUI ACTION

Only if licensed. Separate installer, separate activation, large asset download. Install RoadRunner Scenario too if covered. Launch it once to confirm it opens.

**If RoadRunner is NOT licensed:** stop and tell the team. Fallback F9 applies, and the submission must state plainly that the scenes are not RoadRunner scenes. We do not describe a substitute as the real thing.

---

## STEP D · Open the project

Extract the ZIP somewhere with a **short path and no spaces**, e.g. `D:\sih\SIH_Indian_AV`.

Open MATLAB, then:

```matlab
cd D:\sih\SIH_Indian_AV
```

Use your actual path. Confirm you are in the right place:

```matlab
ls
```

**✅ PASS:** you see `setupPaths.m`, `README.md`, and folders `config`, `planner`, `utils`, `tests`, `scenarios`.

**❌ FAIL:** you see something else → you are in the wrong folder.

---

## STEP E · Path initialisation

```matlab
setupPaths
```

**Expected output** (the path will be yours):

```
SIH_Indian_AV: added 35 source folders to the path.
Project root: D:\sih\SIH_Indian_AV
```

**✅ PASS:** exactly **35** folders.

**❌ FAIL — fewer than 35:** some folders did not survive the transfer. Re-extract the ZIP.

**❌ FAIL — `Undefined function 'setupPaths'`:** you are not in the project folder. `cd` there first.

> ⚠️ **Run `setupPaths` at the start of every MATLAB session.** Nothing else works without it.

---

## STEP F · MATLAB's own static check

This is MATLAB's Code Analyzer — a real parser, far stronger than the pre-flight text analysis already done.

```matlab
checkcode('planner/irpscPlanner.m')
```

Then across the project:

```matlab
files = dir(fullfile(pwd,'**','*.m'));
files = files(~contains({files.folder},'existing_work'));
nIssue = 0;
for k = 1:numel(files)
    f = fullfile(files(k).folder, files(k).name);
    msgs = checkcode(f);
    if ~isempty(msgs)
        fprintf('\n=== %s ===\n', files(k).name);
        for m = 1:numel(msgs)
            fprintf('  line %d: %s\n', msgs(m).line, msgs(m).message);
        end
        nIssue = nIssue + numel(msgs);
    end
end
fprintf('\nTotal Code Analyzer messages: %d across %d files\n', nIssue, numel(files));
```

**What to expect:** I genuinely do not know the number. Static pre-flight found 0 structural defects, but MATLAB's parser checks far more.

**✅ PASS:** no message containing the word **"Parse error"**. Style warnings are fine and expected.

**❌ FAIL:** any **Parse error** → that file will not run. Fix it before continuing, and record what you changed.

**📤 Send back:** the full output of this loop.

---

## STEP G · The smallest smoke test

Work upward. Do not jump to the end.

### G1 · One pure function

```matlab
s = pathArcLength([0 0; 3 4; 3 8])
```

**Expected — exactly:**
```
s =
     0
     5
     9
```

**✅ PASS:** `[0; 5; 9]`.
**❌ FAIL:** anything else, or an error. This is the simplest function in the project; if it fails, something fundamental is wrong. **Send the exact error.**

### G2 · Configuration loads

```matlab
cfg = irpscConfig('village');
disp(cfg.ego.maxSpeed)
```

**Expected:** `11.1000`

### G3 · One geometry test suite

```matlab
results = runtests('testGeometry')
```

**Expected:** 21 tests. I cannot tell you how many pass — **this has never been run.**

**✅ PASS:** all 21 pass.
**⚠️ PARTIAL:** some fail → normal on first run. Record which, and the error.
**❌ FAIL:** the suite will not even start → likely `setupPaths` was not run.

### G4 · The full unit suite

```matlab
results = runAllTests('unit')
```

Runs 5 unit suites. Test counts, verified by counting the test functions in the source — **these are how many tests exist, not how many pass. None has ever run.**

| Suite | Tests |
|---|---|
| `testGeometry` | 21 |
| `testCorridor` | 15 |
| `testPredictionAndRisk` | 17 |
| `testPlannerCore` | 22 |
| `testDecisionAndVehicle` | 17 |
| **Unit total** | **92** |
| `testClosedLoop` (integration, run via `'all'`) | 12 |
| **Grand total** | **104** |

**Expected output ends with:**
```
RESULT: <X> of <Y> passed, <Z> failed, 0 incomplete
```

**✅ PASS:** all pass.
**⚠️ EXPECTED ON FIRST RUN:** some fail. The runner prints every failing test name.
**📤 Send back:** the complete output including every failure.

### G5 · One planning cycle — the real milestone

```matlab
scn = buildScenario('village', 1);
cfg = irpscConfig('village');
ego = makeEgoState(scn.egoStart.pos, scn.egoStart.heading, scn.egoStart.speed);
out = irpscPlanner(scn.grid, ego, [], cfg, []);

fprintf('status      : %s\n', out.status);
fprintf('corridor ok : %d\n', out.corridor.valid);
fprintf('corridor len: %.1f m\n', out.corridor.length);
fprintf('min width   : %.2f m\n', out.corridor.minWidth);
fprintf('quality     : %.2f\n', out.corridor.quality);
fprintf('confidence  : %.2f\n', out.confidence);
fprintf('traj points : %d\n', size(out.traj.pos,1));
```

**✅ PASS looks like:**
- `status` = `ok`
- `corridor ok` = `1`
- `corridor len` > 10 m
- `min width` ≥ 2.8 m
- `quality` and `confidence` between 0 and 1
- `traj points` = 41

**This is the milestone that matters.** It means drivable-space corridor extraction works on an unmarked road — the core IR-PSC claim.

**⚠️ PARTIAL:** runs but `status` is `no_corridor` → corridor extraction has a bug. Debug with:
```matlab
corr = extractCorridor(scn.grid, ego, cfg, []);
figure; hold on; axis equal;
plot(corr.left(:,1),  corr.left(:,2),  'b--');
plot(corr.right(:,1), corr.right(:,2), 'b--');
plot(corr.center(:,1),corr.center(:,2),'k:');
plot(ego.pos(1), ego.pos(2), 'go', 'MarkerSize', 10);
```

**📤 Send back:** the printed values, plus that figure if it fails.

### G6 · Closed loop, short run

```matlab
[log, M] = runScenario('village', @irpscPlanner, [], struct('seed',1,'maxTime',5.0));
fprintf('steps %d | collisions %d | min clear %.2f m | avg speed %.2f m/s\n', ...
        numel(log.t), M.collisionCount, M.minClearance, M.averageSpeed);
```

**✅ PASS:** completes without error and prints four numbers. **Do not judge the numbers yet** — quality assessment comes after the tests pass.

### G7 · Reproducibility — non-negotiable

```matlab
a = runScenario('village', @irpscPlanner, [], struct('seed',7,'maxTime',4.0));
b = runScenario('village', @irpscPlanner, [], struct('seed',7,'maxTime',4.0));
isequal(a.egoPos, b.egoPos)
```

**Expected:** `ans = 1`

**❌ FAIL — `ans = 0`:** 🔴 **STOP.** Something reads global random state. **Every comparative result in this project would be invalid.** Fix before running any experiment.

### G8 · Independent Frenet cross-check

```matlab
exportFrenetReference('python/frenet_reference.csv');
```

Then in a terminal:
```bash
python python/crosscheck_frenet.py python/frenet_reference.csv
```

**✅ PASS:** `RESULT: AGREEMENT within tolerance`

**❌ FAIL:** `DISAGREEMENT` → one implementation is wrong. A sign error in the lateral offset `d` would make the planner deform **toward** hazards rather than away, consistently enough to look deliberate. **Fix before trusting any planning result.**

---

## STEP H · Simulink / Stateflow / RoadRunner

**Only after Steps E–G pass.**

### H1 · Simulink 👤 partly GUI

```matlab
buildClosedLoopModel('sihClosedLoop', irpscConfig('village'))
```

⚠️ **Never executed, and deliberately incomplete.** It creates blocks and sets the solver; it does **not** wire them. Follow `simulink/SIMULINK_ARCHITECTURE.md` to paste the wrapper code and connect the blocks 👤.

**Watch for:** an algebraic-loop error on the ego feedback path. Insert a **Unit Delay** — this is documented and expected.

### H2 · Stateflow 👤 GUI

If Stateflow is licensed, build the chart from `decision/stateflow/STATEFLOW_SPEC.md` 👤, then validate it against `decisionLogic.m`: feed both the same recorded input sequence; **the state sequences must match exactly.**

If **not** licensed, record: **STATEFLOW = NOT VERIFIED · FALLBACK = USED**. `decisionLogic.m` is complete and working.

### H3 · RoadRunner 👤 GUI

Build the two required scenes from `roadrunner/ROADRUNNER_PLAN.md` 👤: the unmarked village road and the unsignalized urban intersection. Prioritise function over scenery.

---

## What to send back to the team

| # | Item | From |
|---|---|---|
| 1 | `version` and full `ver` output | Step E |
| 2 | RoadRunner: licensed yes/no; version if installed | Step C |
| 3 | Code Analyzer output | Step F |
| 4 | `runAllTests('unit')` **complete output including failures** | G4 |
| 5 | The seven printed values from the planning cycle | G5 |
| 6 | Reproducibility result (`1` or `0`) | G7 |
| 7 | Frenet cross-check verdict | G8 |
| 8 | Any error text, **verbatim and complete** | anywhere |
| 9 | The corridor figure, if G5 failed | G5 |

Paste error text as text, not as a screenshot of text — it is searchable and copyable.

---

## 🚫 Do not do this

| Don't | Why |
|---|---|
| **Hide or omit a failure** | A documented failure is useful; a hidden one becomes an unanswerable question in front of judges |
| **Change a test to make it pass** | Unless the test is demonstrably wrong — then say so and explain why |
| **Report "all tests pass" without seeing it** | Only report what you watched happen |
| **Skip to Step H** because E–G are tedious | H cannot work if the core does not |
| **Create a `.slx`/`.sfx`/`.rrscene` by hand** | Fabricated artefacts are worse than missing ones |
| **Record a metric before a run produced it** | `computeMetrics` throws on an empty log by design |
| **Delete `existing_work/`** | 174 passing tests and the Frenet cross-check live there |

---

## Quick reference

```matlab
cd <project folder>
setupPaths                    % every session
runAllTests('unit')           % expect failures first time
runAllTests('all')            % includes reproducibility
demoScenario('village')       % watch it drive
runAllExperiments(1:10)       % only after tests pass
```

---

## Common first-run errors

| Error | Cause | Fix |
|---|---|---|
| `Undefined function 'xyz'` | `setupPaths` not run | Run it |
| `Index exceeds array bounds` | Off-by-one, often at an array end | Check the loop bound |
| `Matrix dimensions must agree` | Row vs column mismatch | **Most likely failure class.** Apply `(:)` |
| `Subscripted assignment between dissimilar structures` | Struct fields differ in name or order | Match the declared field order exactly |
| `Too many output arguments` | Requesting more outputs than the function returns | Check the signature |
| `The value of 'X' is invalid` | An `inputParser` validator rejected a value | Check the argument type |

---

**Nothing in this project is validated until you have run it and seen it pass. Report what actually happens.**
