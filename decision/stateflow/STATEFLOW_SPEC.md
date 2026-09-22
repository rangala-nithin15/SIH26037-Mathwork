# Stateflow Chart Specification — IR-PSC Decision Logic

**Component status:** SPECIFICATION — no Stateflow artefact exists
**Executable reference implementation:** [`decision/decisionLogic.m`](../decisionLogic.m)

---

## Why there is no `.sfx` file in this package

A Stateflow chart is a binary artefact that only Stateflow can create. Stateflow was not installed on the machine where this project was written, so no chart could be built, opened, or verified.

**Fabricating one was not an option.** A hand-written file claiming to be a Stateflow chart would either fail to open or, worse, open as something that had never been checked against the behaviour it claims to implement.

Instead this package provides two things that are genuinely useful:

1. **This specification** — complete enough to build the chart from, with every state, transition, guard and action written out.
2. **`decisionLogic.m`** — a complete, working implementation of exactly this specification in plain MATLAB. It runs today, needs no toolbox, and is unit-tested in `tests/unit/testDecisionAndVehicle.m`.

If Stateflow is licensed on the destination machine, build the chart from this spec and validate it against `decisionLogic.m` — they must agree. If Stateflow is **not** licensed, `decisionLogic.m` is a legitimate `DOCUMENTED FALLBACK` (F3 in [COMPONENT_REGISTER.md](../../docs/COMPONENT_REGISTER.md)) and the project works without Stateflow.

---

## States

| State | Purpose | Speed limit |
|---|---|---|
| `NORMAL_DRIVING` | Clear road, confidence sufficient | `cfg.ego.maxSpeed` |
| `HAZARD_ASSESSMENT` | Something is developing; watch it, ease off | `0.75 × maxSpeed` |
| `PREDICTIVE_AVOIDANCE` | Risk is real; drive the deformed trajectory | `0.55 × maxSpeed` |
| `CONSERVATIVE_DRIVING` | Inputs not trustworthy enough for normal operation | `cfg.decision.conservativeSpeed` |
| `SAFE_STOP` | No feasible path, or risk past the stop threshold | `0` |
| `RECOVERY` | Conditions improved; return deliberately, not instantly | `1.5 × conservativeSpeed` |

`NORMAL_DRIVING` is the default entry state.

### Why `HAZARD_ASSESSMENT` exists as a separate state

Without it the system jumps from full speed straight to hard avoidance. That produces exactly the surge-and-brake behaviour that makes an autonomous vehicle feel unsafe even when it is technically fine. A gentle intermediate response, entered early, is both more comfortable and safer, because it buys time before the situation becomes critical.

---

## Inputs

| Signal | Type | Source |
|---|---|---|
| `risk` | double, 0–1 | `plan.risk` — peak predicted conflict risk |
| `confidence` | double, 0–1 | `plan.confidence` — from `computeConfidence` |
| `ttc` | double, s | `plan.ttc` — from `timeToConflict` |
| `isSafeStop` | boolean | `plan.isSafeStop` |
| `feasible` | boolean | `plan.feasible` |
| `clearanceOk` | boolean | `plan.clearanceOk` |
| `statusCode` | int8 | encoded `plan.status` (see below) |

`statusCode` encoding — Stateflow does not handle strings well, so the planner status is encoded as an integer at the boundary:

| Code | Status |
|---|---|
| 0 | `ok` |
| 1 | `no_corridor` |
| 2 | `deformation_infeasible` |
| 3 | `clearance_failed` |
| 4 | `feasibility_failed` |
| 5 | `no_feasible_candidate` |

## Outputs

| Signal | Type | Meaning |
|---|---|---|
| `stateId` | int8 | 1–6, in the table order above |
| `speedLimit` | double, m/s | Cap applied to the trajectory |
| `useSafeStop` | boolean | Command the safe-stop trajectory |

## Local data (persistent between steps)

| Name | Type | Purpose |
|---|---|---|
| `framesInState` | int32 | Enforces minimum dwell |
| `enterCounter` | int32 | Debounce counter toward a candidate state |
| `candidateId` | int8 | Which state we are debouncing toward |
| `goodFrames` | int32 | Consecutive frames of good conditions, for recovery |

## Parameters (from `cfg.decision` and `cfg.risk`)

`confHigh` 0.70 · `confLow` 0.40 · `riskHazard` 0.25 · `riskAvoid` 0.50 · `riskStop` 0.85 · `debounceEnter` 3 · `debounceExit` 8 · `minDwellFrames` 5 · `recoveryFrames` 10 · `conservativeSpeed` 4.0 · `ttcCritical` 1.5 · `ttcWarning` 3.5

---

## Transition logic

### 1. Emergency override — highest priority, bypasses all debounce

From **any** state to `SAFE_STOP`, evaluated first:

```
[isSafeStop || !feasible || !clearanceOk ||
 risk >= riskStop || ttc <= ttcCritical || statusCode > 0]
```

**This transition ignores both the debounce counter and the minimum dwell timer.** That is the single deliberate exception in the whole chart, and it is justified precisely: a debounce counter must never be able to delay an emergency response. Note that the exception only ever *escalates* — it can never be used to relax a state.

### 2. Desired-state evaluation (before debounce)

Evaluated in strict priority order:

| Priority | Condition | Desired state |
|---|---|---|
| 1 | `confidence < confLow` | `CONSERVATIVE_DRIVING` |
| 2 | `risk >= riskAvoid` | `PREDICTIVE_AVOIDANCE` |
| 3 | `risk >= riskHazard \|\| ttc <= ttcWarning` | `HAZARD_ASSESSMENT` |
| 4 | `confidence < confHigh` | `CONSERVATIVE_DRIVING` |
| 5 | otherwise | `NORMAL_DRIVING` |

Confidence is checked **before** risk. If the inputs are untrustworthy, the risk number computed from them is untrustworthy too, so acting on low confidence takes priority over acting on a risk value we have reason to doubt.

### 3. Recovery gate

If the previous state was `SAFE_STOP`, `CONSERVATIVE_DRIVING` or `PREDICTIVE_AVOIDANCE`, and the desired state is `NORMAL_DRIVING`:

```
if goodFrames < recoveryFrames  ->  desired = RECOVERY
```

`goodFrames` increments only while `confidence >= confHigh && risk < riskHazard && ttc > ttcWarning`, and resets to 0 the moment any of those fails.

### 4. Debounce

Let `severity` rank states as:

```
NORMAL_DRIVING(1) < RECOVERY(2) < HAZARD_ASSESSMENT(3)
  < CONSERVATIVE_DRIVING(4) < PREDICTIVE_AVOIDANCE(5) < SAFE_STOP(6)
```

```
if desired == current:
    enterCounter = 0
else:
    if desired != candidate:  candidate = desired; enterCounter = 1
    else:                     enterCounter++

    needed = (severity(desired) > severity(current)) ? debounceEnter : debounceExit

    if enterCounter >= needed AND framesInState >= minDwellFrames:
        transition to desired
```

**Asymmetric debounce is the core of the design.** Escalation needs 3 frames (0.15 s at 20 Hz); relaxation needs 8 (0.40 s). The system becomes careful quickly and relaxes reluctantly, which is the correct asymmetry for a safety system.

---

## Chart diagram

```
                    ┌──────────────────────────────┐
                    │      NORMAL_DRIVING          │◄────────┐
                    │      limit = maxSpeed        │         │
                    └───────────┬──────────────────┘         │
                risk>=hazard    │                            │ goodFrames >=
                or ttc<=warning │                            │ recoveryFrames
                    ┌───────────▼──────────────────┐         │
                    │    HAZARD_ASSESSMENT         │   ┌─────┴────────────┐
                    │    limit = 0.75*maxSpeed     │   │    RECOVERY      │
                    └───────────┬──────────────────┘   │ limit=1.5*consv  │
                risk>=avoid     │                      └─────▲────────────┘
                    ┌───────────▼──────────────────┐         │
                    │   PREDICTIVE_AVOIDANCE       │─────────┤
                    │   limit = 0.55*maxSpeed      │         │
                    └───────────┬──────────────────┘         │
                                │                            │
                    ┌───────────▼──────────────────┐         │
   conf<confLow ───►│   CONSERVATIVE_DRIVING       │─────────┤
                    │   limit = conservativeSpeed  │         │
                    └───────────┬──────────────────┘         │
                                │                            │
                    ┌───────────▼──────────────────┐         │
                    │        SAFE_STOP             │─────────┘
                    │        limit = 0             │
                    └──────────────────────────────┘
                                ▲
                                │  EMERGENCY OVERRIDE from ANY state
                                │  (bypasses debounce and dwell)
```

---

## Building the chart

1. `sfnew sihDecisionChart`
2. Add the six states with the names above; set `NORMAL_DRIVING` as the default.
3. Add the inputs, outputs and local data listed above.
4. Add the emergency transition from each state to `SAFE_STOP`, and set its **execution order to 1** so it is evaluated first.
5. Implement the desired-state evaluation and debounce in the chart's `during` action, or in a preceding MATLAB Function block that outputs `desiredId` — the latter is simpler and keeps the chart readable.
6. Set the chart sample time to `cfg.sim.dt`.

## Validating the chart against the reference implementation

The chart and `decisionLogic.m` must produce identical state sequences. To check:

1. Record `[risk, confidence, ttc, isSafeStop, feasible, clearanceOk, statusCode]` from a `runScenario` log.
2. Feed that recorded sequence to both the chart and `decisionLogic.m`.
3. Compare the `stateId` sequences. **They must match exactly.**

Any divergence is a bug in one of them. Find out which before using either.

---

**Status: SPECIFICATION ONLY. No Stateflow chart has been created, opened, executed or verified by this project.**
