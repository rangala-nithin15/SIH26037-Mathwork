# Judge Demo Guide

Running the demonstration, and answering the hard questions honestly.

---

## The one rule

**Never claim more than you have measured.**

Judges at this level have seen many teams overclaim. A team that says "we simulate perception geometrically, we haven't trained a detector, and here's exactly what that means for our results" is more credible than one claiming 95% detection accuracy it cannot defend. Your honesty is an asset — use it deliberately.

---

## Before the demo

- [ ] `setupPaths` runs cleanly
- [ ] `runAllTests('unit')` passes — **know your real numbers**
- [ ] `runDemo` runs start to finish on the demo laptop (time it — MATLAB speed varies)
- [ ] `runDemo(struct('videoFile','results/irpsc_demo.mp4'))` has produced a **backup video**
- [ ] Each `demoScenario` runs without error
- [ ] `runAllExperiments` has been run; you have real results
- [ ] **A recorded video exists as backup.** Live demos fail.
- [ ] You can explain every component you show
- [ ] You know which components are SIMPLIFIED and FALLBACK

**If a demo fails live:** say "let me show you the recording" and move on. Do not debug in front of judges.

---

## The demo, in eight minutes

### 1 · The problem (45 s)

> "Indian roads often have no usable lane markings. Conventional planners detect lanes, build a lane-centre reference, and choose among fixed lateral offsets. On a road with no markings, that first step fails and everything downstream collapses. The usual answer is a fallback mode — which means the system's normal mode is the one that doesn't work here."

### 2 · The idea (45 s)

> "We inverted the hierarchy. Drivable space is the primary constraint; lane markings are an optional cue. There's no lane-detection step to fail — we measure free space directly, infer the boundaries from it, and follow the middle of whatever space actually exists. An unmarked village road isn't a degraded case for us. It's the ordinary case."

### 3 · The live 3D demonstration (3 min) — Phase 2

```matlab
setupPaths
runDemo
```

This is the primary demo: one road, every behaviour, planned live. For the presentation itself, record the run beforehand and play it back smoothly — replay shows exactly what the planner logged, at a steady frame rate:

```matlab
[log, M] = runDemo(struct('visible', false));
save('results/demo_run.mat', 'log', 'M');
replayDemo('results/demo_run.mat')            % 30 FPS; struct('fps', 60) etc.
```

If asked whether the replay is "real": "Every frame is the logged planner output for that step. Between the 20 Hz recorded states the car's picture is interpolated so it does not jump; no decision or trajectory is changed." Be ready to run `runDemo` live to show the same behaviour being planned in real time, slower.

Narrate the sequence as it happens. The right-hand panel reads top to bottom: the state word (CRUISE / SLOW / AVOID / STOP), risk and confidence as LOW / MEDIUM / HIGH, then the decision log, which spells out each step as `[DETECT]`, `[PREDICT]`, `[PLAN]`, `[DECISION]`:

1. **Marked approach, then markings end.** "The white lines are drawn for you. The planner never reads them — the label on the road says where they end, and nothing about the planning changes."
2. **Moderate pothole ahead.** "Detected by the simulated camera and LiDAR, confirmed by the pothole tracker after three observations, severity estimated from depth. Violet cells are its cost in the planner. The cyan path bends around it and comes back."
3. **Minor pothole narrower than the wheel track.** "Straddled — no reaction needed, which is what a good driver does."
4. **Slow bicycle with an oncoming bus.** "No room to pass: the car follows at the bicycle's speed — the trajectory turns amber — until the bus has gone, then overtakes."
5. **Market stretch with a parked truck and a cart.** "The green corridor is measured free space; watch it shift right around the truck. The path follows the corridor."
6. **Severe pothole, then a broken patch across the road.** "The severe one is avoided; the patch cannot be, so the car slows to the pothole speed cap and continues."
7. **Cow walks into the road and stands.** "Predicted occupancy grows, the car yields with a controlled stop, and continues once the animal has moved on. Emergency braking is reserved for genuinely imminent conflicts."
8. **Pedestrian crossing.** "Same mechanism."

Press **2** for the overview camera and **3** for top-down when useful; **p** switches to the planning view (corridor, trajectory, hazards only) and **f** back to full. **Space** pauses so you can point at the risk cells, the corridor and the trajectory. The colours keep one meaning throughout: **green = space the car may use, cyan = what it intends to do, orange/red = predicted hazard, red = stop.**

> Say plainly: "This is a schematic 3D view drawn with MATLAB graphics, not RoadRunner. Every object and line on screen is the simulation's state at this step."

### 3b · The village road (2 min)

```matlab
runDemo(struct('scenario', 'village'))     % or the 2D view: demoScenario('village')
```

Point at things as they happen:

> "The grey area is drivable space. **Notice there are no lane markings anywhere in this scene** — there's nothing for a lane detector to find.
>
> The dashed blue lines are the road boundaries. We found those by casting rays perpendicular to our path until free space ends. That's measurement, not detection.
>
> The dotted line is the corridor centre — our preferred path. The solid green line is what we'll actually drive. **The gap between them is the contribution**: the planner bending around a predicted hazard while staying inside the corridor.
>
> The orange ellipses are predicted occupancy. They grow with time and with uncertainty. The motorcycle's are much larger than the bicycle's, because a motorcycle can change direction faster — and because they're larger, the planner automatically gives it more room. There's no code anywhere that says 'if uncertain, be careful'. It falls out of the representation."

### 4 · Side by side (2 min)

```matlab
demoScenario('market', 'baseline', struct('seed',3));
demoScenario('market', 'irpsc',    struct('seed',3));
```

> "Same seed, so both planners face an identical world — same actor timings, same sensor noise, same missed detections. Any difference is the planner.
>
> The baseline has five fixed lateral offsets. On a street where the usable centre wanders every few metres, the right position is almost never one of the five. Watch it stop."

Then show the real numbers from your comparison run.

### 5 · The emergency (1 min)

```matlab
demoScenario('cattle')
```

> "Everything's clear, the vehicle's settled at speed, and at six seconds cattle step out. Watch the state change in the title bar. Emergency conditions bypass the debounce entirely — a hysteresis counter must never delay an emergency response. That's the one deliberate exception in the state machine, and it only ever escalates."

### 6 · The evidence (1.5 min)

```matlab
printComparison(results);
```

> "We built the conventional planner too, and gave it every fair advantage — same corridor, same vehicle limits, same scoring function, same fallback. Then we ran both across ten seeds on five scenarios.
>
> We also ran a five-way ablation, disabling one component at a time, because beating a baseline shows the system works but not which part is doing the work."

Show the numbers as they are, including anywhere IR-PSC did not win.

### 7 · The honest close (30 s)

> "To be clear about what this is: simulation, with a geometric sensor model and a kinematic vehicle model. We haven't trained a detector. It isn't safety certified and we make no collision-avoidance guarantee. What we've shown is that reordering the planning hierarchy — drivable space first, continuous path, uncertainty that changes behaviour, confidence that degrades autonomy early — measurably changes how the planner handles unstructured roads. And here's exactly how much."

---

## The hard questions

**"What's actually new? Frenet coordinates and Kalman filters are decades old."**

> "You're right, and we don't claim any of them. Ray casting, road-aligned coordinates, Gaussian uncertainty, dynamic programming, EKF tracking — all standard, all cited as supporting technologies. What's ours is the ordering: drivable space as the primary constraint instead of lane markings, a continuous preferred path instead of fixed offsets, uncertainty that widens the hazard region so being unsure automatically produces caution, and confidence in our own inputs driving early degradation. Whether that ordering helps is an empirical question, so we built the conventional planner and measured it."

**"Have you tested on a real vehicle?"**

> "No. This is simulation research. We make no real-world claim, and our metrics carry a provenance record saying so."

**"How accurate is your detection?"**

> "We don't have detection. We simulate it geometrically from ground truth with realistic failure modes — field of view, occlusion, range-dependent noise, missed detections, false positives, class confusion. That lets us study planning under imperfect perception, which is what we set out to do. We can't quote a detection accuracy, and we won't."

**"Can you detect auto-rickshaws and cattle?"**

> "Not today. Those aren't classes any off-the-shelf detector provides — it'd need a custom annotated dataset of Indian traffic. Our planning and prediction handle all nine road-user classes; the perception layer simulates their detection. We flag that gap explicitly in the code, in `objectClasses.m`."

**"Where are the RoadRunner scenes?"**

> "Not built. RoadRunner wasn't available on our development machine, and we didn't fabricate scene files. We have full specifications for both required scenes — geometry, features, actors, integration steps — and our MATLAB scenarios were designed as their blueprint. It's our largest outstanding gap and we're not hiding it."

**"Is it real-time?"**

> "We measure replan latency and report the 95th percentile, because the tail governs deadlines, not the average. But these are MATLAB interpreted timings on a laptop. They're valid for comparing our two planners; they're not evidence of embedded real-time performance. That would need generated code on target hardware."

**"What if the planner is wrong?"**

> "Three independent safeguards. The clearance check is purely geometric against current positions, deliberately separate from prediction, so a prediction failure can't disable it. The feasibility check makes sure the vehicle can actually drive the path — otherwise the controller cuts the corner and every safety check we did was on a path we never drove. And if no feasible trajectory exists, the decision logic commands a safe stop. It's called safe stop because that's the intent and the manoeuvre, not because we claim stopping is always safe."

**"Your actors don't react to you. Isn't that unrealistic?"**

> "Completely correct, and it's the biggest limitation. On Indian roads negotiation is constant. We scripted them deliberately so both planners face an identical, repeatable challenge — a reactive actor that politely yields would quietly flatter whichever planner it yielded to. The cost is that we cannot demonstrate negotiation, and we don't claim to."

**"How do I know your baseline isn't a straw man?"**

> "Fair question. It gets the same corridor, the same vehicle limits, the same scoring function and the same safe-stop fallback. Giving it our corridor actually makes the comparison conservative — it isolates prediction, deformation and confidence rather than also crediting us for not needing lane markings. It understates our advantage rather than inflating it. The code is in `baselinePlanner.m` and the header explains exactly what it does and doesn't do."

**"What would prove you wrong?"**

> "We wrote that down before running anything. If the baseline matches us on village and market, the continuous-corridor claim isn't doing work. If `no_prediction` matches `full`, the prediction machinery is unjustified. If we win clearance only by driving much slower, we've traded away the objective rather than solving it. It's in `VALIDATION_PLAN.md`, section 7."

**"Did all your tests pass?"**

> Say the real number. *"X of Y pass. These N fail, and here's why."* Never say "all tests pass" unless you have watched them all pass.

---

## Things never to say

| ❌ Don't say | ✅ Say instead |
|---|---|
| "Zero accidents" | "Zero collisions in these simulation runs" |
| "Guaranteed collision-free" | "Predictive risk assessment plus independent geometric checks" |
| "Real-time" | "5 Hz replanning in simulation; not validated on target hardware" |
| "Our detector recognises cattle" | "Our planner handles cattle; detection is simulated" |
| "Validated" | "Tested in simulation" |
| "Safety certified" | Nothing. Never imply it. |
| "It always works" | "Here's where it fails, and why" |

---

## If asked for the code

Point to these, in this order:

1. `README.md` — including the "read this first" warnings
2. `docs/COMPONENT_REGISTER.md` — every component's REAL / SIMPLIFIED / FALLBACK label
3. `planner/irpscPlanner.m` — the header states plainly what is and is not claimed
4. `REQUIREMENTS_TRACEABILITY.md` — every requirement, honestly statused, including the one not met

A judge who opens the code and finds the limitations documented in the source, in the same words you used out loud, will trust everything else you said.
