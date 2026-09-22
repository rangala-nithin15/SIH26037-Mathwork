# RoadRunner Scene Plan

**Component status:** SPECIFICATION — **no RoadRunner file exists in this package**

---

## Why this directory contains no scene files

RoadRunner scenes (`.rrscene`), projects (`.rrproj`) and scenarios (`.rrscenario`) are binary artefacts produced by RoadRunner, which is a visual 3D editor. RoadRunner was not installed on the machine where this project was written.

**No RoadRunner file has been created, and none has been fabricated.** A hand-written file with a RoadRunner extension would not open, and passing one off as a real scene would be exactly the kind of misrepresentation this project refuses to make.

## What the SIH problem statement requires

> At least two detailed RoadRunner scenes, including a village road and an urban intersection.

**This requirement is currently NOT MET, and cannot be met from this laptop.** It must be completed on the destination machine, and it depends on RoadRunner being licensed — which is still an open question (see `ENVIRONMENT_REPORT.md`, risk R2).

If RoadRunner turns out not to be licensed, fallback **F9** applies: build the road networks with `drivingScenario` or OpenDRIVE and **state explicitly in the submission that these are not RoadRunner scenes**. Do not describe a substitute as the real thing.

## What this package provides instead, today

The five scenarios in `scenarios/` are MATLAB geometric abstractions. They exercise the planner correctly and run with base MATLAB alone. They are labelled `SIMPLIFIED SIMULATION` everywhere they appear, and they are **not** RoadRunner scenes.

They also serve a second purpose: their geometry is the specification for the RoadRunner scenes below. Building the RoadRunner version of a scene that already runs in MATLAB is far easier than designing it from nothing.

---

## Scene 1 — Unmarked village road (REQUIRED)

**MATLAB reference:** `scenarios/village/scenarioVillage.m`

| Property | Value |
|---|---|
| Length | ~120 m |
| Width | 3.8–5.6 m, **irregular** |
| Lane markings | **NONE.** Not faded — absent. |
| Surface | Unpaved or poorly paved; visible edge break-up |
| Geometry | Gentle S-curve, radius roughly 40–80 m |

**Must include:**
- Ill-defined road edges — the transition to the verge should be gradual, not a kerb. This is the whole point of the scene: there is no crisp boundary for a perception system to latch onto.
- A pinch point around 56–66 m where usable width drops to ~3.8 m.
- A parked truck partly occupying the road at the pinch point.
- Roadside debris and vegetation encroaching irregularly.
- Potholes as surface irregularities (see the note on potholes below).
- Buildings/walls set back irregularly from the road.

**Actors:** oncoming motorcycle, slow bicycle in the direction of travel, pedestrian walking along the verge who moves further into the road.

**Why this scene matters:** it is the clearest demonstration of the central claim. A lane-marking-based planner has literally nothing to work with. Film the demo here.

---

## Scene 2 — Busy unsignalized urban intersection (REQUIRED)

**MATLAB reference:** `scenarios/urban_intersection/scenarioUrbanIntersection.m`

| Property | Value |
|---|---|
| Type | Four-way, **no traffic signal** |
| Main road | ~100 m through, 7.2 m wide |
| Cross road | ~60 m, 7.2 m wide |
| Junction mouth | Widened, ~17 m across |
| Markings | Worn or absent; **no stop lines, no crossings** |

**Must include:**
- Buildings at all four corners, set close enough to genuinely restrict sightlines into the junction. The occlusion is a feature, not scenery.
- No lane discipline markings anywhere.
- Roadside shops, parked two-wheelers, informal vendor positions.
- Uneven road surface through the junction.

**Actors:** cross traffic from both directions timed to conflict, an auto-rickshaw merging from a side road without signalling, an oncoming bus, a filtering motorcycle, a pedestrian crossing away from any marked point.

**Why this scene matters:** it demonstrates prediction. The hazards approach from the side and are occluded until late, so reacting to current positions is provably too slow.

---

## Scenes 3–5 (recommended, not required by the problem statement)

| Scene | MATLAB reference | Key features |
|---|---|---|
| Highway merge | `scenarios/highway_merge/scenarioHighwayMerge.m` | 300 m, 11 m wide, on-ramp joining at a shallow angle, crash barriers, slow truck and bus |
| Dense market | `scenarios/market/scenarioMarket.m` | 80 m, 3.0–5.2 m usable, stalls encroaching from both sides, dense pedestrians |
| Cattle crossing | `scenarios/cattle_crossing/scenarioCattleCrossing.m` | 160 m open rural road, roadside vegetation, wide enough that avoidance is a real option |

---

## A note on potholes

The problem statement asks for potholes and road irregularities "where practical". Be precise about what is actually being modelled.

**What can be done honestly:**
- Model a pothole geometrically in RoadRunner as a surface depression, so it is visible in the scene and in camera/LiDAR sensor returns.
- Mark severe potholes as **non-drivable cells** in the occupancy grid. The planner then routes around them exactly as it routes around any other obstruction — no special-case code, which is a genuine strength of the drivable-space approach.

**What must NOT be claimed:**
- That the vehicle model responds to driving over a pothole. It does not. The kinematic bicycle model has no suspension, no tyre model and no vertical dynamics (see `vehicle/dynamics/bicycleModelStep.m`). Modelling ride response over a pothole would require Vehicle Dynamics Blockset and a road-surface model.
- That pothole *detection* works. No detector has been trained for it.

State the distinction plainly in the submission: potholes are modelled as **planning obstacles**, not as **dynamic disturbances**.

---

## MATLAB ↔ RoadRunner integration — to be verified

**Every item below is `NOT VERIFIED`.** None has been tested, because neither MATLAB nor RoadRunner is installed here. Do not assume any of it works until you have run it.

| Step | What to check | How |
|---|---|---|
| 1 | RoadRunner is licensed | mathworks.com → My Account → My Software |
| 2 | RoadRunner Scenario is licensed | Same page — it is a separate item |
| 3 | The MATLAB API connects | Try `rrApp = roadrunner(projectFolder)` |
| 4 | Scenes can be opened from MATLAB | `openScene(rrApp, sceneName)` |
| 5 | Export works | Export to OpenDRIVE (`.xodr`) and/or FBX |
| 6 | Import into a driving scenario | `roadNetwork(scenario, 'OpenDRIVE', file)` — needs Automated Driving Toolbox |
| 7 | Co-simulation | RoadRunner Scenario ↔ Simulink |

**Function names above are from MathWorks documentation and have not been executed.** Verify each against `doc` on your installed release before relying on it. If a name has changed between releases, trust `doc`, not this file.

---

## Getting from a RoadRunner scene to this planner

The planner consumes an **occupancy grid**, not a road network (see `utils/makeOccupancyGrid.m`). The bridge that needs writing on the destination machine is:

```
RoadRunner scene
    → export OpenDRIVE (.xodr)
    → import into a drivingScenario           [Automated Driving Toolbox]
    → simulate sensors, or sample road geometry
    → rasterise drivable surface into an occupancy grid
    → irpscPlanner(grid, ego, obstacles, cfg, state)
```

Only the final step exists today, and it works. The rasterisation step is the missing piece, and it is small — `scenarios/buildRoadGrid.m` already does exactly this from a centreline and width profile, so adapting it to consume OpenDRIVE geometry rather than a hand-written centreline is a contained job.

**Design note:** the planner was deliberately given an occupancy-grid interface rather than a road-network interface. That is what lets it work identically on a RoadRunner scene, a `drivingScenario`, or the hand-built MATLAB scenarios — and it is also what makes "no lane markings required" true at the interface level, not just in the algorithm.

---

**Status: SPECIFICATION ONLY. No RoadRunner scene, project or scenario file has been created or verified. The SIH requirement for two detailed RoadRunner scenes is NOT MET and must be completed on the destination machine.**
