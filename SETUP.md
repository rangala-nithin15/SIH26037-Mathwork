# Setup

Written for someone new to MATLAB. Every step says what to do and why.

---

## What you are setting up

A MATLAB project that simulates an autonomous vehicle planning its way along Indian roads that have no lane markings.

**Nothing here has ever been run.** MATLAB was not installed on the machine where this code was written. Expect errors on first run — that is normal for several thousand lines written without an interpreter, and fixing them is the first task, not a sign something is wrong.

---

## Step 0 · Check your MathWorks licence

**Do this before installing anything.** It determines everything else.

1. Sign in at **mathworks.com → My Account → My Software**
2. Write down:
   - Licence type (Student / Campus-Wide / SIH-provided / Trial)
   - The exact list of licensed products
   - **Whether RoadRunner is included — yes or no**

RoadRunner has historically been licensed separately from ordinary MATLAB suites, and the SIH problem statement requires two detailed RoadRunner scenes. Ask your SIH coordinator and faculty mentor before buying anything — participating teams are often issued access, and college Campus-Wide Licences usually carry the widest set.

---

## Step 1 · Install MATLAB

**The good news: the entire planner runs on base MATLAB.** No toolbox is needed for the planner, prediction, decision logic, controller, dynamics, scenarios, metrics, experiments or tests.

So you can install MATLAB alone, run the whole system, and add toolboxes later as licensing allows.

| Priority | Product | What it unlocks here |
|---|---|---|
| **Essential** | MATLAB | Everything |
| Recommended | Simulink | The closed-loop block model (`buildClosedLoopModel.m`) |
| Recommended | Stateflow | The decision chart (a working MATLAB version already exists) |
| Optional | Automated Driving, Navigation, Sensor Fusion and Tracking | Upgrades to fallback components |
| Optional | Computer Vision, Deep Learning, Lidar | Real perception instead of simulated |
| Optional | Model Predictive Control | Upgrade from pure pursuit |
| Optional | Vehicle Dynamics Blockset | Upgrade from the bicycle model |
| **For SIH** | RoadRunner + RoadRunner Scenario | The two required scenes |

Install to a drive with plenty of free space. RoadRunner asset libraries and scene projects grow.

---

## Step 2 · Get the project onto the machine

Unzip it somewhere with a **short path and no spaces**, e.g. `D:\sih\SIH_Indian_AV`.

Long paths and spaces cause avoidable trouble with MATLAB and build tools.

---

## Step 3 · Verify what you actually got

Open MATLAB and run these in the Command Window. **Test rather than assume** — this is the step people skip and regret.

```matlab
version
```
The MATLAB release. Write it down.

```matlab
ver
```
Lists **every installed product**. This is ground truth — not the installer's promises, not this document.

```matlab
license('test','Simulink')
license('test','Stateflow')
license('test','Automated_Driving_Toolbox')
```
`1` means available, `0` means not.

**Send me the `ver` output.** I will size the next phase to what you actually have.

---

## Step 4 · Set up the path

```matlab
cd D:\sih\SIH_Indian_AV
setupPaths
```

You should see something like:

```
SIH_Indian_AV: added 35 source folders to the path.
Project root: D:\sih\SIH_Indian_AV
```

**Run `setupPaths` at the start of every MATLAB session.** It is the only setup step that is strictly required.

*What it does:* MATLAB only finds functions in folders on its "path". This project spreads code across many folders, so `setupPaths` adds them all. It works out where it is from its own location, so no path is hard-coded and the project runs from anywhere on any machine.

---

## Step 5 · Run the tests

```matlab
runAllTests('unit')
```

**Expect failures.** This code has never been executed. First-run failures are normal and expected.

The output ends with something like:

```
RESULT: 71 of 78 passed, 7 failed, 0 incomplete
FAILED TESTS:
  - testCorridor/testCorridorOnStraightRoad
  ...
```

Work through them one at a time. Most first-run failures in MATLAB code are:

| Symptom | Usual cause |
|---|---|
| `Undefined function or variable` | `setupPaths` not run, or a typo in a function name |
| `Index exceeds array bounds` | An off-by-one, often at an array end |
| `Matrix dimensions must agree` | A row vector where a column was expected (or vice versa) |
| `Too many output arguments` | Calling a function with more outputs than it returns |

**Record the real numbers in `PROJECT_STATUS.md` once they pass.** Never report this suite as passing until you have seen it pass.

---

## Step 6 · Watch it drive

```matlab
demoScenario('village')
```

This runs the unmarked village road and animates it. What to look for:

- **Dashed blue lines** — the corridor boundaries, found by ray-casting free space. There are no lane markings anywhere in the scene; those edges came from measuring where the drivable surface ends.
- **Dotted grey line** — the corridor centreline, the preferred path.
- **Solid green line** — the actual planned trajectory. **The gap between the dotted and solid lines is the deformation** — the planner bending around a predicted hazard.
- **Orange ellipses** — predicted occupancy. They grow with time and with uncertainty. Note how much larger they are for the motorcycle than for a slow bicycle.
- **The title bar** — live speed, risk and confidence.

Then try the others:

```matlab
demoScenario('urban')     % occluded crossing traffic
demoScenario('highway')   % shallow-angle merge at speed
demoScenario('market')    % the hardest one
demoScenario('cattle')    % emergency stop
```

---

## Step 7 · Optional extras

### Create a MATLAB Project

```matlab
createProject
```

Convenience only — automatic path management, dependency analysis, source-control integration. **`setupPaths` is sufficient without it.**

### Build the Simulink model

```matlab
buildClosedLoopModel('sihClosedLoop', irpscConfig('village'))
```

⚠️ **Never executed. Incomplete.** It creates the blocks; you must wire them by hand following `simulink/SIMULINK_ARCHITECTURE.md`. The pure-MATLAB pipeline runs the same algorithms today without Simulink.

### Set up Python (supporting, not required)

Install **Python 3.11 or 3.12** — not 3.14, which is likely newer than MATLAB accepts.

```bash
cd existing_work/mathworks_scraper
pip install -r requirements.txt
python -m pytest
```

That README claims 174 tests. **That is its claim, not a measured result.** Run it and report the real number.

The independent Frenet cross-check:

```matlab
exportFrenetReference('python/frenet_reference.csv');
```
```bash
python python/crosscheck_frenet.py python/frenet_reference.csv
```

This compares the MATLAB road-aligned transform against an independently written Python implementation. Worth doing: a sign error in the lateral offset would make the planner steer *into* hazards, consistently enough to look deliberate.

---

## Step 8 · Run the experiments

Only after the tests pass.

```matlab
runAllExperiments(1:3)     % quick check that it works end to end
runAllExperiments(1:10)    % the real thing
```

With 10 seeds that is 400 simulations and will take a while. Start with 3.

---

## Troubleshooting

**`Undefined function 'irpscPlanner'`** — run `setupPaths` first.

**`Undefined function 'ver' for 'simulink'`** — Simulink is not installed. Fine; only `buildClosedLoopModel` needs it.

**Everything is slow** — `runAllExperiments` runs hundreds of full simulations. Use fewer seeds. For a single run, `runScenario` with a short `maxTime` is fast.

**Figures do not appear** — check you are not in `-nodisplay` mode. `demoScenario` needs a display.

**Out of memory** — do not run RoadRunner and a full simulation at the same time. Export scenes, then simulate.

---

## What to report back

1. MATLAB release and the full `ver` output
2. RoadRunner: licensed, yes or no
3. `runAllTests('unit')` — **the real numbers**, including failures
4. Anything that failed and why

Then the next phase can be planned around what actually exists.
