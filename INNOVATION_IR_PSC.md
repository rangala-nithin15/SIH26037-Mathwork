# IR-PSC — Indian-Road Predictive Safety Corridor

**The contribution, stated precisely — including what is *not* claimed.**

---

## 1. What is claimed, and what is not

### Not claimed as novel

Every technique used here is standard and well documented. **None of the following is our innovation:**

ray casting · Frenet / road-aligned coordinates · Gaussian uncertainty propagation · dynamic programming · Kalman filtering · global nearest-neighbour association · sensor fusion · pure pursuit · the kinematic bicycle model · occupancy grids · YOLO · LiDAR · radar · MPC · Stateflow · MATLAB · Simulink · RoadRunner

These are supporting technologies. Anyone claiming otherwise would be wrong, and this document exists partly so that nobody on this team accidentally does.

### What *is* claimed

**The hierarchy.** Specifically: which quantity is treated as primary, and what the system does when it is unsure.

Four properties, in order of importance:

1. **Drivable space is the primary planning constraint. Lane markings are an optional cue.** The planner never asks where the lane is. It is not that lane detection is used and then a fallback engages when it fails — there is no lane detection in the pipeline at all.

2. **The preferred path is continuous.** The corridor centreline moves smoothly as the road bends, narrows and widens. It is not a choice among a handful of fixed lateral offsets.

3. **Uncertainty changes behaviour rather than being reported.** A vague prediction inflates the hazard ellipse, and the planner gives it more room. There is no special-case code for "uncertain obstacles" — the mechanism falls out of the representation.

4. **Confidence in the system's own inputs is measured and acted upon.** Autonomy degrades *before* a failure, not after one.

None of these four is individually unprecedented in the literature. The claim is that assembling them in this order, with drivable space at the root, produces a planner suited to unstructured Indian roads in a way that a lane-first planner is not — and that the difference is measurable, which is what the baseline comparison and ablation study exist to establish.

---

## 2. The problem with the conventional approach

A conventional structured-road planner does roughly this:

```
detect lane markings → build a lane-centre reference → generate candidate
trajectories at fixed lateral offsets → score them against obstacles → pick one
```

Every step of that assumes something an Indian road may not provide.

| Assumption | Reality on an unmarked road |
|---|---|
| Lane markings exist and are detectable | Often absent entirely, not merely faded |
| The road has a fixed number of lanes | Usable width varies continuously |
| Lateral position is quantised to lane centres | The right position is wherever there is room |
| Other road users stay in lanes | They do not, and are not expected to |
| The road edge is a crisp boundary | It is a gradual transition to verge, ditch, or stalls |

When lane detection fails, such a planner has no reference path, and everything downstream collapses. The usual response is a fallback mode — which means the system's normal operating mode is the one that does not work here.

**IR-PSC inverts this.** There is no lane-detection step to fail. Free space is measured directly, boundaries are inferred from it, and the reference path is the middle of whatever space actually exists. An unmarked village road is not a degraded case; it is the ordinary case.

---

## 3. The thirteen steps

### Steps 1–2 · Drivable space → corridor

`seedCenterline.m` marches forward from the vehicle, repeatedly fanning out candidate headings, ray-casting each into free space, and stepping along the one that maximises clearance minus a turn penalty.

The turn penalty matters. Without it the march oscillates between two near-equal openings and produces a zig-zag no later smoothing can fully repair.

`extractCorridor.m` then takes each station on that seed and casts one ray left and one right, **perpendicular to the path, until free space ends**. Those two hits *are* the road edges. No painted line is involved anywhere.

The centreline is then recomputed as the midpoint of the two eroded boundaries. This step matters more than it looks: the greedy seed hugs whichever side had more room, and taking the midpoint recentres the corridor. That is what makes the preferred path *road-following* rather than *free-space-hugging*.

**A ray that runs to maximum range without hitting anything means the edge was not observed** — open field, missing return, sensor gap. Treating that as a confident boundary would be dishonest, so it lowers `corridor.quality`, which flows into planner confidence, which changes behaviour. Honesty about a measurement is wired directly into the control path.

### Steps 3–5 · Track, predict, and represent uncertainty

Prediction starts from the simplest defensible model — damped constant acceleration, with class-dependent speed caps so a pedestrian is never predicted at 30 m/s. It is fully interpretable and needs no training data.

Its known weakness is that it cannot anticipate a turn that has not started. Two mechanisms cover that:

**Uncertainty growth.** Treating unknown acceleration as white noise gives

```
var(t) = var_pos0 + var_vel0·t² + (σ_a²·t⁴)/4
```

The t⁴ term is what makes far-future predictions appropriately vague.

**Class-dependent lateral spread.** This is where "these road users do not follow lanes" is actually encoded, as extra lateral standard deviation per second of horizon:

| Class | m/s | Class | m/s |
|---|---|---|---|
| bus, truck | 0.15 | auto-rickshaw, bicycle | 0.55 |
| car | 0.25 | motorcycle | 0.70 |
| pushcart | 0.45 | pedestrian | 0.90 |
| | | **animal** | **1.10** |

A cow is the least predictable thing on the road, and the model says so.

`irregularMotionModel.m` adds explicit alternative manoeuvres a lane-following model cannot express: `cutLeft`, `cutRight`, `cross`, `decelerate`. A pedestrian standing at the road edge is weighted far more likely to cross than one walking away from it. These hypotheses are then **folded back into lateral uncertainty** as weighted spread, so the downstream risk computation stays a single Gaussian-shaped occupancy per time step — far cheaper than carrying every hypothesis through the planner, while still widening the danger zone where a swerve is plausible.

> **Honest label:** these weights are hand-designed heuristics, not probabilities learned from data. They are interpretable, defensible, and deliberately conservative — they add hypotheses rather than removing them. Learning them from recorded Indian traffic is future work.

### Step 6 · Predicted-occupancy risk

Each predicted road user occupies, at each instant, an ellipse centred on its predicted mean and aligned with its predicted heading, with semi-axes

```
a_long = egoRadius + obsRadius + n·σ_long
a_lat  = egoRadius + obsRadius + n·σ_lat
```

**This is the mechanism by which uncertainty changes behaviour.** An erratic motorcycle has a large σ_lat, so its ellipse is wide, so the planner routes further around it — with no code anywhere that says "if uncertain, give more room."

Risk from several road users combines as `1 − Π(1 − rₖ)`, not as a sum. Two separate 0.6 hazards must not produce a risk of 1.2.

`lateralRiskGrid.m` makes this **time-aware**, which is easy to miss and important. Station *i* is reached at roughly `s(i)/v` seconds, so a pedestrian predicted to cross in two seconds only endangers the stations the ego reaches at around two seconds. A purely spatial obstacle map would brake for a hazard that will have cleared the road long before the vehicle arrives.

### Steps 7–8 · Continuous path, deformed around hazards

The corridor centreline is the preferred path. `deformTrajectory.m` chooses a lateral offset at every station by minimising

```
J = Σ [ w_risk·R(i,dᵢ) + w_dev·dᵢ² ] + Σ w_smooth·(dᵢ − dᵢ₋₁)² + w_anchor·(d₁ − d₀)²
```

Because the smoothness term couples only neighbouring stations, **dynamic programming finds the exact global optimum** over the discretised lateral grid in O(N·K²). No local minima, no gradient tuning, no iteration count to guess.

Each term earns its place:

- `w_risk` — avoid hazards. The point.
- `w_dev` — prefer the corridor centre. Without it the path drifts to whichever side is momentarily emptier and wanders.
- `w_smooth` — punish sharp lateral changes, which are uncomfortable and often infeasible.
- `w_anchor` — start where the vehicle actually is, so the controller is not handed a step it cannot execute.

Cells outside the corridor arrive as `Inf` and are simply never selected. **No weighting of the other terms can ever buy a path off the road.** If an entire station is blocked, the problem is infeasible and that is reported — which is what triggers a safe stop.

### Steps 9–11 · Margins, smoothing, feasibility

`corridorBounds.m` computes, per station, how far the path may move while keeping the whole vehicle body inside the drivable corridor. Where the corridor is narrower than the vehicle plus margins, both bounds collapse to zero — reported honestly rather than widened to something drivable, because a corridor narrower than the car is a fact the decision logic needs.

`checkClearance.m` is an **independent** geometric check against current obstacle positions and the occupancy grid. It is deliberately separate from the predictive risk assessment: prediction can be wrong, and where things are *right now* is the last line of defence. Keeping it independent means a prediction failure cannot silently disable it.

`checkFeasibility.m` verifies curvature, lateral acceleration, longitudinal acceleration, jerk and steering angle. A planner that skips this produces beautiful paths the car cannot drive; the controller then cuts the corner, the driven path differs from the checked path, and **every safety check performed on the plan becomes meaningless.**

### Steps 12–13 · Confidence and graceful degradation

`computeConfidence.m` combines four genuinely measurable quantities:

| Component | Weight | What it measures |
|---|---|---|
| Corridor quality | 0.35 | Lookahead achieved, width adequacy, boundary rays that found an edge |
| Perception confidence | 0.25 | Mean track confidence |
| Track maturity | 0.20 | A scene of one-frame-old tracks is a scene we do not understand |
| Prediction sharpness | 0.20 | Inverse of mean end-of-horizon uncertainty |

> **Honest label:** a designed heuristic, not a calibrated probability. A confidence of 0.8 does **not** mean an 80% chance of being right. It is an internal, monotonic quality signal. Calibrating it against outcomes is future work.

`decisionLogic.m` acts on it through six states with **asymmetric debounce**: escalation needs 3 consecutive frames, relaxation needs 8. The system becomes careful quickly and relaxes reluctantly. Emergency conditions bypass both debounce and minimum dwell — the one deliberate exception, and it only ever escalates.

---

## 4. How the claim gets tested

A contribution never compared against the obvious alternative is an assertion, not a result.

**`baselinePlanner.m`** implements the conventional recipe properly: five fixed lateral offsets, obstacles treated as static, no confidence awareness. It gets the same corridor, the same vehicle limits, the same scoring function, the same safe-stop fallback. Any difference comes from the planning approach.

Giving the baseline the corridor is deliberate and makes the comparison **conservative** — it isolates the effect of prediction, continuous deformation and confidence, rather than conflating them with the separate benefit of not needing lane markings. It understates the full IR-PSC advantage rather than inflating it.

**The ablation study** goes further. Beating a baseline shows the system works; it does not show which part is doing the work. Five variants each disable exactly one component. If disabling prediction barely changes anything, prediction is not earning its complexity — and that should be reported, not assumed away.

**What would falsify this contribution**, stated in advance so the test is meaningful:

- If the baseline matches IR-PSC on the village and market scenarios, the "continuous corridor" claim is not doing work.
- If `no_prediction` matches `full`, the prediction machinery is unjustified complexity.
- If `no_confidence` matches `full`, confidence-aware degradation is decoration.
- If IR-PSC achieves clearance only by driving far slower, it has not solved the problem — which is why average speed and completion rate are always reported next to the safety metrics.

See [`VALIDATION_PLAN.md`](VALIDATION_PLAN.md).

---

## 5. Limitations

- **Greedy seed search.** `seedCenterline` can be fooled by a wide dead end that looks better than the narrow correct road. Mitigated by boundary ray-casting correcting the lateral placement, and by replanning several times per second. Not eliminated.
- **Single-hypothesis risk.** Manoeuvre hypotheses are folded into one Gaussian per time step. Cheap and effective, but it cannot represent a genuinely bimodal future (left *or* right, not the middle).
- **No interaction model.** The planner does not reason about how other road users will react to *it*. On Indian roads, negotiation is real and constant. This is the largest gap.
- **No follower awareness.** Emergency braking does not account for a vehicle behind.
- **Heuristic weights throughout.** Class risk weights, lateral spread rates, confidence weights and cost weights are all engineering judgement, not fitted to data.
- **Scripted actors.** Scenario actors do not react to the ego vehicle, so negotiation cannot be demonstrated even in principle.

---

## 6. If someone asks "so what is actually new?"

Answer it directly:

> The pieces are all standard, and we say so. What is ours is the ordering: we treat drivable space as the primary constraint instead of lane markings, we keep the preferred path continuous instead of quantising it to fixed lane offsets, we let prediction uncertainty widen the hazard region so that being unsure automatically produces more caution, and we measure confidence in our own inputs and slow down before things go wrong instead of after.
>
> Whether that ordering actually helps is an empirical question, so we built the conventional planner too and ran both on identical scenarios with identical noise, plus a five-way ablation to show which component is doing the work. The numbers are in the report, including the cases where it does not help.
