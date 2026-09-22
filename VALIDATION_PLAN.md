# Validation Plan

How this system gets validated, what counts as success, and **what would prove the contribution wrong**.

---

## 0. Current status

**Nothing has been validated. Nothing has been run.** This document is the plan, written before any result exists — deliberately, so the success criteria cannot be adjusted after seeing the numbers.

That ordering matters. Criteria chosen after the fact are not criteria.

---

## 1. Levels of validation

| Level | What it establishes | Status |
|---|---|---|
| L1 · Unit | Individual functions are mathematically correct | ❌ never run |
| L2 · Integration | The closed loop runs and is reproducible | ❌ never run |
| L3 · Scenario | The system handles the five SIH scenarios | ❌ never run |
| L4 · Comparative | IR-PSC differs measurably from the conventional approach | ❌ never run |
| L5 · Ablation | Each component contributes | ❌ never run |
| L6 · Real-world | — | **Out of scope. Not attempted, not claimed.** |

L6 is listed only to be explicit that it is absent. Nothing in this project is evidence about real vehicles on real roads.

---

## 2. L1 — Unit validation

**Command:** `runAllTests('unit')`

Tests are written against cases with analytically known answers, plus the degenerate inputs that break naive implementations.

| Suite | Covers | Notable cases |
|---|---|---|
| `testGeometry` | Arc length, resampling, heading, curvature, Frenet | Circle curvature = 1/R; coincident points give 0 not NaN; ±π wrap; **lateral sign convention** |
| `testCorridor` | Grid, ray casting, corridor extraction | Out-of-grid is occupied; corridor width matches the built road; centreline is centred; blocked road is invalid |
| `testPredictionAndRisk` | Prediction, uncertainty, risk, TTC | Uncertainty grows; pedestrians spread wider than buses; risk ≤ 1 with many hazards; vulnerable classes score higher |
| `testPlannerCore` | Deformation, safety, planner | DP avoids hazards; forbidden cells never chosen; stopping distance = v²/2a; **both planners share an interface** |
| `testDecisionAndVehicle` | Decision logic, control, dynamics | **No chatter under alternating input**; emergency bypasses debounce; recovery is not instant; integrator clamped |

**Pass criterion:** all unit tests pass.

**Expect failures on the first run.** Fix them and record the real numbers.

### The tests that matter most

- **`testWrapToPiIntervalIsHalfOpen` and `testProjectPointLateralSign`.** A sign error in the lateral offset would make the planner deform *toward* hazards, consistently enough to look deliberate.
- **`testNoChatterUnderAlternatingInput`.** Feeds risk oscillating across the threshold for 40 frames and requires fewer than 6 state changes. Without working hysteresis this fails loudly.
- **`testRiskAlwaysInUnitInterval`.** Six overlapping hazards must not produce risk > 1.

---

## 3. L2 — Integration validation

**Command:** `runAllTests('integration')`

| Property | Test | Why it matters |
|---|---|---|
| All five scenarios build | `testScenariosAllBuild` | Also checks the ego starts in drivable space |
| A run completes | `testShortRunCompletesWithoutError` | — |
| **Same seed → identical run** | `testRunIsReproducibleWithSameSeed` | **Without this, every comparison is invalid** |
| Different seed → different noise | `testDifferentSeedsGiveDifferentNoise` | Guards against the seed doing nothing |
| Both planners run everywhere | `testBothPlannersRunOnAllScenarios` | — |
| Metrics cannot be invented | `testMetricsAreComputedNotInvented` | `computeMetrics` throws on an empty log |
| Ablation switches do something | `testAblationSwitchesChangeBehaviour` | Guards against a study that measures nothing |

Note what is **not** asserted: no performance threshold. Asserting "clearance must exceed 0.5 m" would bake in a result nobody has measured. These tests check *properties*, not *results*.

---

## 4. L3 — Scenario validation

Each scenario is run with both planners across ≥10 seeds.

| Scenario | Primarily tests | Expected discriminator |
|---|---|---|
| A · Village | The central claim — **no lane markings exist at all** | Baseline's five fixed offsets rarely line up with the gap beside the parked truck on a road of varying width |
| B · Urban intersection | Prediction under occluded crossing traffic | Baseline treats obstacles as static; a car arriving in 1.5 s is invisible to it until it is there |
| C · Highway merge | Early decisions at speed | Shallow-angle merge; lateral separation shrinks slowly right up until it does not |
| D · Market | Everything at once, at low speed | Corridor centre wanders constantly; expect baseline `no_feasible_candidate` failures |
| E · Cattle crossing | Emergency escalation and safe stop | Tests the path that bypasses debounce |

**Per-scenario criteria:**

1. The run completes without error.
2. Failure statuses, if any, are explicable — a `no_corridor` in a genuinely blocked situation is correct behaviour, not a bug.
3. The vehicle makes progress. A planner that never moves is safe and useless, which is why average speed and completion rate are always reported next to safety metrics.

---

## 5. L4 — Comparative validation

**Command:** `runComparison(allScenarios, 1:10)`

Both planners face byte-identical worlds: same seed, same actor timings, same sensor noise, same missed detections, same false positives. Enforced by construction in `runComparison.m` — it is not possible to accidentally compare across different noise realisations.

The baseline is given the **same corridor** as IR-PSC. That is deliberate and makes the comparison **conservative**: it isolates the effect of prediction, continuous deformation and confidence, rather than conflating them with the separate benefit of not needing lane markings. It understates the full advantage rather than inflating it.

### Metrics reported, and how to read them

| Metric | Direction | Note |
|---|---|---|
| Worst minimum clearance | higher better | **Worst case across runs, not the mean** — the mean of a minimum is not a safety statement |
| Collision count | lower better | Distinct events, not time steps |
| Completion rate | higher better | Goal reached **and** no collision |
| Average speed | higher better | The counterweight to every safety metric |
| Smoothness | lower better | RMS d(curvature)/ds |
| Planner failures | lower better | Broken down by cause |
| Replan latency (mean and p95) | lower better | Relative comparison only |

### ⚠️ The success criterion, fixed in advance

IR-PSC is considered to add measurable value **if and only if**, across ≥10 seeds on all five scenarios, it shows a clear advantage on at least one safety metric (worst clearance or collision count) **without a material loss** in completion rate or average speed.

**Buying safety with speed is not success.** A planner that crawls everywhere trivially achieves excellent clearance. That is why the criterion is a joint one.

---

## 6. L5 — Ablation validation

**Command:** `runAblation(allScenarios, 1:10)`

Beating a baseline shows the system works. It does not show which part is doing the work — a component can contribute nothing while the whole still wins.

| Variant | Disables | Expected effect if the component matters |
|---|---|---|
| `no_prediction` | Future occupancy | Worse in urban and cattle, where hazards develop over time |
| `no_uncertainty` | Sigma-inflated ellipses | Less margin around erratic road users; worst in market |
| `no_confidence` | Confidence-aware slowdown | Higher speed, worse clearance under poor perception |
| `no_deformation` | Local hazard avoidance | Falls back to centreline-only; more emergency stops |
| `no_smoothing` | Temporal blending | Mainly a smoothness-metric effect |

**Interpretation, stated before the results exist:** a variant showing near-zero deltas everywhere means that component is **not earning its complexity in these scenarios**, and that must be reported as a finding rather than assumed away.

---

## 7. What would falsify the contribution

Written in advance. A claim that cannot be falsified is not a claim.

| Finding | What it would mean |
|---|---|
| Baseline matches IR-PSC on **village** and **market** | The continuous-corridor claim is not doing work. These are the scenarios it should most help. |
| `no_prediction` matches `full` | The prediction machinery is unjustified complexity in these scenarios. |
| `no_confidence` matches `full` | Confidence-aware degradation is decoration. |
| `no_deformation` matches `full` | The DP deformation is not earning its cost. |
| IR-PSC wins on clearance **only by driving much slower** | It has not solved the problem, it has traded away the objective. |
| High `lateralTrackingError` | The vehicle is not following the checked path, so **every safety check performed on the plan is meaningless**. |
| Reproducibility test fails | All comparative results are invalid and must be withdrawn. |

**If any of these occurs, report it.** A negative result honestly reported is worth more than a positive one that does not survive a judge's question.

---

## 8. Known threats to validity

Things that could make the results look better than the system deserves:

| Threat | Why it matters | Mitigation |
|---|---|---|
| **Scripted actors do not react** | Real road users would sometimes yield, making avoidance easier than modelled | Same for both planners; comparison stays fair. But **no negotiation claim may be made** |
| **Simulated perception** | Real detection failures differ in kind, not just degree | Never quote detector performance; both planners see the identical sensor stream |
| **Kinematic vehicle model** | No tyre slip; the vehicle always achieves what is commanded | Feasibility limits are applied; **no handling or stability claim** |
| **Self-designed scenarios** | We built the tests our system takes | Scenarios were specified from the SIH problem statement, and the baseline is given every advantage that is fair |
| **Self-designed baseline** | A weak baseline would flatter IR-PSC | Baseline gets the same corridor, limits, scoring and fallback. It is a faithful implementation, not a straw man |
| **Heuristic weights** | Class weights, spread rates, cost weights are judgement, not fitted | Documented as such throughout; ablation shows which actually matter |
| **Small seed count** | 5 seeds is not 500 | `aggregateMetrics` reports std and n, so spread and sample size cannot be omitted |

---

## 9. Reporting rules

Every reported number must carry:

1. **Provenance** — `M.provenance` records the sensor and vehicle model and sets `realWorldClaim = false`.
2. **Sample size and spread** — mean without std and n is not a claim.
3. **The simulation caveat**, printed automatically by `printComparison`.

Forbidden phrasings:

- ❌ "zero collisions" without "in simulation, under a simplified sensor model"
- ❌ "collision-free" — the system reduces risk, it does not guarantee freedom
- ❌ "real-time" — no target-hardware timing exists
- ❌ "detects auto-rickshaws / cattle" — no detector has been trained
- ❌ "safety certified" or "validated" — neither is true

---

## 10. Order of work on the destination machine

1. `runAllTests('unit')` — fix failures, record real numbers
2. `runAllTests('all')` — including reproducibility
3. `demoScenario` on each of the five — sanity-check the behaviour visually
4. `runComparison(all, 1:3)` — check the pipeline end to end
5. `runComparison(all, 1:10)` — the real comparison
6. `runAblation(all, 1:10)` — the component study
7. Build the two required RoadRunner scenes — **the outstanding SIH gap**
8. Re-run scenarios A and B against the RoadRunner scenes
9. Write up, including every negative result
