# Adaptive Path Planning for Autonomous Vehicles on Unstructured Indian Roads

**Smart India Hackathon 2026 · Problem Statement ID: 26037 · MathWorks**

Core idea: **IR-PSC (Indian-Road Predictive Safety Corridor)**

> *Lane markings are an optional cue. Drivable space is the primary planning constraint.*

---

## The Problem

Most autonomous-driving planners assume clean lane markings, clear road edges and traffic that follows lane discipline. Indian roads rarely offer any of that. Lane markings are often faded or missing, road boundaries are unclear, and the ego vehicle shares space with cars, buses, trucks, two-wheelers, auto-rickshaws, pedestrians, pushcarts and cattle, many of which merge informally or cross without warning. Potholes and irregular road geometry make it worse.

A lane-following planner has nothing to follow in these conditions. So the question the vehicle needs to answer at every moment is: **where can I safely drive right now?**

## Our Solution

We built **IR-PSC**, a planner that works from drivable space instead of lanes. Each planning cycle, it:

1. Finds the drivable corridor directly from free space (no lane markings needed) and extracts its boundaries and centreline.
2. Tracks surrounding road users and predicts where they will be over the next few seconds, with uncertainty that grows over time and is larger for unpredictable users such as pedestrians, two-wheelers and animals.
3. Builds a risk grid from predicted occupancy, time-to-conflict and pothole cost.
4. Deforms the preferred path around hazards using dynamic programming, while keeping boundary and obstacle clearance.
5. Checks the trajectory against vehicle limits, scales speed to its confidence, and slows down or stops safely when no safe path exists.

It runs as a full closed-loop simulation (sensors → fusion → tracking → prediction → planning → decision → control → vehicle dynamics) and draws every step live in a 3D view. A conventional fixed-candidate planner is included as a baseline, so the two can be compared on the same road with the same seed.

---

## Tech Stack

| Category | Technology | Used for |
|---|---|---|
| Language | MATLAB | Entire planner, simulation and visualisation |
| Core platform | Base MATLAB (no toolboxes required) | IR-PSC pipeline runs without any toolbox |
| Planning | Frenet-frame trajectories, dynamic programming | Corridor extraction and trajectory deformation |
| Prediction | Constant-velocity model + uncertainty growth + irregular-motion model | Short-term motion of road users |
| Perception (simulated) | Camera, LiDAR and radar models, multi-sensor fusion, multi-object tracking | Detecting and tracking road users and potholes |
| Decision logic | 6-state machine (Stateflow spec included) | CRUISE / SLOW / AVOID / STOP / RESUME behaviour |
| Vehicle model | Kinematic bicycle model + controllers | Closed-loop vehicle motion |
| Visualisation | MATLAB graphics (3D viewer), VideoWriter | Live demo, replay and video recording |
| Extensions (planned) | Simulink, Stateflow, RoadRunner | Model-based integration and detailed scenes |
| Supporting tools | Python | Cross-checking the Frenet reference |

---

## How to Run the Project in MATLAB

### Step 1: Install MATLAB

1. Go to **https://www.mathworks.com/downloads** and sign in, or create a free MathWorks account. Students can usually get a licence through their college's MATLAB Campus-Wide Licence; use your college email address.
2. Download the MATLAB installer for your operating system (Windows, macOS or Linux).
3. Run the installer and sign in with your MathWorks account.
4. When asked which products to install, **MATLAB** is the only one this project needs. Simulink and Stateflow are optional, for future integration.
5. Finish the installation and open MATLAB. Any recent release works (R2021a or newer is recommended).

### Step 2: Download this project

**Option A: download a ZIP (easiest)**

1. Open this repository on GitHub.
2. Click the green **Code** button, then **Download ZIP**.
3. Extract the ZIP to a short path with no spaces, for example `D:\sih\SIH26037-Mathwork`.

**Option B: clone with Git**

```bash
git clone https://github.com/rangala-nithin15/SIH26037-Mathwork.git
```

### Step 3: Open the project folder in MATLAB

In the MATLAB **Command Window**, go to the folder you extracted or cloned:

```matlab
cd D:\sih\SIH26037-Mathwork
```

(Replace the path with your own. You can also browse to the folder in MATLAB's **Current Folder** panel.)

### Step 4: Set up the paths

Run this **once every time you open MATLAB**:

```matlab
setupPaths
```

It adds all the project folders to MATLAB's path. You should see a message like:

```
SIH_Indian_AV: added N source folders to the path.
Project root: D:\sih\SIH26037-Mathwork
```

### Step 5: Run the main demo

```matlab
runDemo
```

A window opens and the live closed-loop simulation starts on the main demo road. It covers every behaviour in one run: avoiding and straddling potholes, following a slower vehicle, shifting the corridor in a market, stopping for cattle, and yielding to a pedestrian.

**Keyboard controls** (click the figure first):

| Key | Action |
|---|---|
| `1` / `2` / `3` | Chase / overview / top-down camera |
| `f` / `p` / `s` / `m` | Full / planning / sensing / minimal view |
| `space` | Pause or resume |
| `q` | Stop |

### Step 6: Run the five SIH scenarios

```matlab
demoScenario('village')    % A. Unmarked village road, no lane markings at all
demoScenario('urban')      % B. Busy unsignalised urban intersection
demoScenario('highway')    % C. Highway merge with a slow vehicle
demoScenario('market')     % D. Dense market with mixed traffic
demoScenario('cattle')     % E. Sudden cattle crossing, emergency stop
```

You can also open any scenario in the full 3D viewer:

```matlab
runDemo(struct('scenario', 'village'))
```

### Step 7 (optional): Compare with the baseline planner

Run the same road and the same seed with a conventional planner, then with IR-PSC:

```matlab
demoScenario('market', 'baseline')
demoScenario('market', 'irpsc')
```

### Step 8 (optional): Record a video or replay smoothly

```matlab
runDemo(struct('videoFile', 'results/irpsc_demo.mp4'))   % record while running
```

For a smooth 30 FPS playback, record once without a window and replay it:

```matlab
[log, M] = runDemo(struct('visible', false));
save('results/demo_run.mat', 'log', 'M');
replayDemo('results/demo_run.mat')
```

---

## What You See in the Demo

| On screen | Meaning |
|---|---|
| White car with cyan ring | The ego vehicle |
| Cyan ribbon | The trajectory the car is following right now (amber when following or blocked, red during a safe stop) |
| Teal / green band | Drivable corridor found from free space |
| Yellow → red cells | Predicted collision risk |
| Violet cells | Pothole cost |
| Orange-red ellipses | Predicted positions of road users at 1, 2 and 3 s |
| Yellow boxes | Tracked objects |
| Red dashed line | A candidate path the planner rejected |
| Right panel | Vehicle state, risk, confidence, time-to-collision, speed and a decision log |

The decision log explains each action step by step, for example: `[DETECT] Cattle ahead` → `[PREDICT] Path conflict in 2.1 s` → `[PLAN] Holding back` → `[DECISION] STOP`.

---

## Project Structure

```
SIH26037-Mathwork/
├── setupPaths.m        ← run this first
├── config/             tunable parameters and scenario profiles
├── utils/              geometry and occupancy-grid helpers
├── planner/
│   ├── irpscPlanner.m  the IR-PSC pipeline
│   ├── IR_PSC/         corridor, risk, deformation, safety, trajectory
│   └── baseline/       conventional planner for comparison
├── prediction/         motion prediction with uncertainty
├── perception/         simulated detection, fusion and tracking
├── sensors/            camera / LiDAR / radar configuration
├── decision/           decision logic + Stateflow specification
├── vehicle/            controller and vehicle dynamics
├── scenarios/          the five SIH scenarios + demo road
├── metrics/            evaluation metrics
├── visualization/      3D viewer and plots
├── scripts/            runDemo, demoScenario, replayDemo
├── simulink/           Simulink model builder and architecture
└── results/            output from your runs
```

---

## Limitations

This is a simulation prototype, not a real-vehicle system. The sensors are geometric models rather than trained detectors, so classes like auto-rickshaws, pushcarts and cattle come from the scenario definitions, not from a real camera. The vehicle model is kinematic (no tyre slip or suspension), and scenario actors are scripted, so they don't react to or negotiate with the ego vehicle. Confidence is a designed heuristic, not a calibrated probability. Simulink, Stateflow and RoadRunner models are specified in the repository but not yet built as `.slx`, `.sfx` or scene files.

## Future Work

Next steps are training a detector on the India Driving Dataset (IDD) so perception runs on real video, building the Simulink and Stateflow models from the included specifications, creating detailed RoadRunner scenes for the SIH scenarios, and making scenario actors interactive so negotiation and yielding can be tested.

---

## Team
LEXICORE
Smart India Hackathon 2026, Problem Statement 26037 (MathWorks).
