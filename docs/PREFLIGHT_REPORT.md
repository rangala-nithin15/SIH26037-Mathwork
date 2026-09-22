# Pre-Flight Report

**Date:** 2026-09-07
**Machine:** `LAPTOP-PPU6OF7R`, Windows 11 build 10.0.26200.9168
**Purpose:** Verify everything that can be verified *before* reaching a machine with MATLAB.

---

## ⚠️ Scope — what this report is and is not

**MATLAB is not installed on this machine.** Nothing in section B is MATLAB execution.

Section A **is** real execution: the Python suite genuinely ran, and its numbers are measured.

Section B is **static text analysis only**. It catches a narrow class of defects — parse-level and structural problems. It cannot catch wrong maths, wrong logic, wrong array orientation, wrong indexing, or a planner that runs but plans badly. **A clean static result is not evidence that the MATLAB code works.**

---

# A · Python test results — EXECUTED ✅

## Command

Determined from the project's own `pytest.ini` and `README.md`:

```bash
cd existing_work/mathworks_scraper
python -m pytest
```

Executed as `python -m pytest -p no:cacheprovider` with `PYTHONDONTWRITEBYTECODE=1`, so the run left no artefacts in the tree. Configuration applied from `pytest.ini`: `testpaths = tests`, `pythonpath = . tests python_port`, `addopts = -q --strict-markers`.

## Environment

| Item | Value |
|---|---|
| Interpreter | Python 3.14.3 (`C:\Python314`) |
| pytest | 9.1.1 |
| numpy | 2.5.2 · requests 2.34.2 · beautifulsoup4 4.15.0 · lxml 6.1.2 · responses 0.26.3 |
| `curl_cffi` | **absent** — declared optional in `requirements.txt`; the suite is unaffected |

## Results

```
174 passed in 1.48s
```

| Outcome | Count |
|---|---|
| **Collected** | **174** |
| **Passed** | **174** |
| Failed | 0 |
| Errors | 0 |
| Skipped | 0 |
| xfail / xpass | 0 |
| Warnings | 1 (benign — see below) |
| **pytest exit code** | **0** |

### Per-file breakdown

| File | Tests |
|---|---|
| `tests/test_parser.py` | 52 |
| `tests/test_fetcher.py` | 36 |
| `tests/test_exporters_and_translate.py` | 32 |
| `tests/test_python_port.py` | **31** ← the Frenet motion model |
| `tests/test_transports.py` | 23 |
| **Total** | **174** |

### Failures

**None.** No tracebacks to report.

### Warning

One `XMLParsedAsHTMLWarning`, raised from `mwscraper/parser.py:160` during `test_parser.py::TestEdgeCases::test_non_html_input_degrades_gracefully`.

This is **not a defect**. The test deliberately feeds XML to an HTML parser to prove the parser degrades gracefully; the warning is the code doing exactly what the test checks. `pytest.ini` filters it during normal runs, and it surfaces only under `-W always`.

### Two log lines that look like errors but are not

`-rA` output contains two lines beginning `ERROR mwscraper.scraper:scraper.py:70 Failed to scrape a: 404`. These are **captured application log output** from a test exercising 404 handling, not pytest errors. The authoritative counts are `174 passed`, `0 failed`, `0 error`, and exit code `0`.

### Reproducibility

Run twice, identical result (174 passed, 1.61 s and 1.48 s).

## Verdict on the README's claim

The README states *"174 tests, fully offline"*.

> **VERIFIED.** Exactly 174 tests, all passing, with no network access. This claim was previously carried as an unverified assertion throughout the project documentation. It is now measured.

The suite also runs entirely offline: `tests/fixtures/frenet_page.html` (95,470 bytes) supplies the saved page, which is why preserving that fixture mattered.

### Caveat

This ran on **Python 3.14.3**. `SETUP.md` recommends **3.11 or 3.12** for the destination machine, because 3.14 is likely newer than any current MATLAB release accepts for a MATLAB↔Python bridge. Python 3.12 is present here but has no packages installed, so the suite has **not** been verified on 3.12. Re-run there after `pip install -r requirements.txt`.

---

# B · MATLAB static analysis — NOT EXECUTION

## Method

A purpose-built conservative analyser over the **86 project `.m` files**. The 10 third-party MathWorks reference listings in `existing_work/` were **excluded from the quality verdict**, as instructed — they are documentation excerpts, not our code.

The analyser tokenises MATLAB properly: line and block comments, single- and double-quoted literals, `...` continuations, and the transpose-versus-string-quote ambiguity. Getting that last one wrong is the main source of false positives in naive MATLAB linters, and it produced one in the first pass here.

## Checks performed

| # | Check | Result |
|---|---|---|
| 1 | Obviously invalid MATLAB syntax patterns | ✅ none |
| 2 | Unbalanced `()` `[]` `{}` | ✅ none |
| 3 | `function`/`end` block structure | ✅ none |
| 4 | Declaration vs filename mismatch | ✅ none (86/86 match) |
| 5 | Chained index on call — `foo(x)(end)` | ✅ none |
| 6 | Undefined local helper references | ✅ none unresolved |
| 7 | Caller/definition argument-count mismatch | ✅ none |
| 8 | Invalid file/function naming | ✅ none |
| 9 | Hard-coded absolute paths | ✅ none |
| 10 | Placeholder/TODO where implementation required | ✅ none |
| 11 | Missing referenced files/dependencies | ✅ none |
| 12 | Duplicate/conflicting function names | ✅ none |
| 13 | Toolbox functions in files declaring base MATLAB | ✅ none |
| 14 | Documentation/component-status inconsistencies | ✅ none |
| 15 | *(added)* struct-array `(end+1)` field-order mismatch | ✅ none |
| 16 | *(added)* variables shadowing builtins | ⚠️ **1 found — fixed** |

Supplementary: every documentation link resolves; every `setupPaths` folder exists; all 86 functions are referenced or are known entry points (no orphans).

## Findings

### GENUINE DEFECT — 1, fixed

#### `MEDIUM` · `perception/fusion/fuseDetections.m:66` · builtin shadowing

**Issue.** A local variable was named `all`, shadowing MATLAB's `all()` builtin for the remainder of the function.

**Why it matters.** The code as written *did* work — the only `all(...)` function call is at line 143 inside `combineGroup()`, a separate function scope. But it is fragile in a way that fails confusingly: anyone later adding `all(someLogical)` inside `fuseDetections` would silently get an indexing operation on a struct array instead of a logical reduction. MATLAB's own Code Analyzer flags this pattern.

**Fix applied.** Renamed the variable to `allDets` at all six sites (lines 66, 71, 78, 94, 95, 107 of the original), with a comment recording why. The builtin call at line 143 was **correctly left untouched** — different scope, genuinely the builtin.

**Risk of the fix:** minimal. A local-variable rename confined to one function, mechanically verified by re-running every check.

### FALSE POSITIVES — 35, all analyser defects

The first pass reported 1 CRITICAL, 25 HIGH and 9 MEDIUM. **Every one was a bug in my analyser, not in the project code.** Each was traced to the specific source line before any change was made, and the analyser — not the project — was corrected.

| Reported | Count | Actual cause |
|---|---|---|
| `CRITICAL` unbalanced `[` in `buildClosedLoopModel.m` | 1 | Tokeniser treated the quote in `[modelName '/' bName]` as a **transpose** because the previous *significant* character was alphanumeric. MATLAB's rule uses the *immediately preceding* character: a quote after whitespace starts a string. The MATLAB code is correct and idiomatic. |
| `HIGH` hard-coded absolute paths (6) | 6 | Regex `[A-Za-z]:\\` matched `:\n` inside `fprintf` format strings — `'Next steps:\n'` matched as `s:\`. Fixed with a preceding-word-character guard. |
| `HIGH` block-structure mismatch (19) | 19 | The `end` counter only matched lines that were *exactly* `end`, so single-line forms like `if ~ok, list{end+1} = 'x'; end` counted an opener but no end. Replaced with depth-aware token counting: an `end` at bracket depth 0 terminates a block; an `end` inside `()`/`[]`/`{}` is the index keyword. |
| `MEDIUM` unresolved calls (9) | 9 | Incomplete builtin whitelist. All nine are legitimate: `dot`, `inf`, `meshgrid`, `isletter`, `mat2str`, `deg2rad` (base MATLAB since R2015b, **not** Mapping Toolbox), plus `addPath`/`addStartupFile`/`removePath` — `matlab.project.Project` methods used only in `createProject.m`, which correctly declares *"Requires: MATLAB R2019a or later"*. |

**This is worth stating plainly:** the first pass looked alarming, and none of it was real. Reporting those 35 as defects would have sent you chasing nothing.

## Final result

```
Analysing 86 project .m files (excluding 10 third-party reference files)

CRITICAL  0
HIGH      0
MEDIUM    0
LOW       0
INFO      0
```

Builtin-shadowing check: `-> none found`.

## NOT STATICALLY DETERMINABLE — REQUIRES MATLAB

The following cannot be settled without an interpreter, and are **not** claimed clean:

- Whether any file actually parses (only MATLAB's parser decides)
- Runtime array-orientation errors (row vs column) — the single most likely failure class in this codebase
- Runtime dimension mismatches in matrix operations
- Whether `interp1` receives strictly increasing sample points at run time
- Whether struct-array growth succeeds with runtime-constructed values
- Whether `inputParser` validators accept the values actually passed
- Numerical correctness of any algorithm
- Convergence, stability, or whether the planner plans *well*
- Whether toolbox functions exist on the target release
- Any Simulink, Stateflow or RoadRunner behaviour

---

# C · Files modified

**One file.**

| File | Change | Lines |
|---|---|---|
| `perception/fusion/fuseDetections.m` | Renamed local variable `all` → `allDets` to stop it shadowing the `all()` builtin; added an explanatory comment | 66–111 |

**Nothing else was touched.** No architecture change, no rebuild, no folder-structure change, no new ZIP.

Verified unchanged: 86 project `.m`, 10 reference `.m`, 19 `.py`, 18 `.md`. The preserved `existing_work/` tree and the original `mathworks_scraper/` outside the project are both byte-for-byte intact. The Python runs left no `__pycache__` or `.pytest_cache` behind.

---

# D · Remaining risks

| # | Risk | Severity | Note |
|---|---|---|---|
| 1 | **The MATLAB code has never been parsed by MATLAB** | 🔴 **HIGH** | Static analysis covers parse-level structure only. Expect runtime errors on first execution. |
| 2 | Array orientation (row vs column) | 🟡 MEDIUM | The most likely failure class. Mitigated by consistent use of `(:)` and `validateattributes`, but not provable statically. |
| 3 | `interp1` on non-increasing samples | 🟡 MEDIUM | Guarded by `keep = [true; diff(s) > 1e-9]` in several places; not provable statically. |
| 4 | Struct-array growth with runtime values | 🟡 MEDIUM | Field order verified statically; runtime construction is not. |
| 5 | Numerical/algorithmic correctness | 🟡 MEDIUM | Entirely unverified. This is what the unit suite exists to establish. |
| 6 | Toolbox function availability on the target release | 🟡 MEDIUM | The planner needs none; only Simulink/Stateflow/RoadRunner paths are exposed. |
| 7 | Python suite not verified on 3.12 | 🟢 LOW | Verified on 3.14 only. |
| 8 | `buildClosedLoopModel.m` is incomplete by design | 🟢 LOW | Documented: it creates blocks but does not wire them. |

---

# E · What MUST be verified once MATLAB is installed

In order. Do not skip ahead.

1. **`setupPaths`** — confirm it adds all 35 folders and reports the right root.
2. **`runAllTests('unit')`** — the first real proof the code parses and computes correctly. **Expect failures.** Record the real numbers.
3. **`runAllTests('integration')`** — especially `testRunIsReproducibleWithSameSeed`. **If that fails, every comparative result in this project is invalid.**
4. **`demoScenario('village')`** — visual sanity check on corridor extraction.
5. **`exportFrenetReference` + `crosscheck_frenet.py`** — the independent cross-check on the road-aligned transform. A sign error there would make the planner deform *toward* hazards.
6. **`ver`** — record actual toolbox availability; update `PROJECT_STATUS.md`.
7. **`runComparison`** then **`runAblation`** — only after tests pass.
8. **Simulink / Stateflow / RoadRunner** — only if licensed.

**Until step 2 has genuinely passed, no claim about MATLAB code correctness may be made anywhere in this project.**

---

# F · What can already be considered structurally clean

Evidence-backed, with the limits of each stated:

| Claim | Evidence | Strength |
|---|---|---|
| The preserved Python suite passes: **174/174** | Executed twice, exit code 0 | ✅ **Measured fact** |
| The Frenet motion model works: **31 tests pass** | Part of the above | ✅ **Measured fact** |
| The suite runs fully offline | No network; fixture-driven | ✅ **Measured fact** |
| All 86 `.m` files: balanced delimiters | Depth-aware tokeniser | ⚠️ Static only |
| All 86: consistent `function`/`end` structure | Depth-aware token counting | ⚠️ Static only |
| All 86: filename matches declared function | Signature parse | ⚠️ Static only |
| No chained-index-on-call syntax errors | Pattern scan | ⚠️ Static only |
| No duplicate/shadowing function names | Cross-file index | ⚠️ Static only |
| No hard-coded absolute paths | Guarded regex | ⚠️ Static only |
| No placeholders or TODOs | Marker scan | ⚠️ Static only |
| No toolbox calls in base-MATLAB files | Toolbox function table | ⚠️ Static only |
| Every documentation link resolves | Link check | ✅ Verified |
| Every `setupPaths` folder exists | Path check | ✅ Verified |
| All 86 functions referenced; no orphans | Call-graph scan | ✅ Verified |
| No variables shadow builtins | Scope-aware check | ⚠️ Static only |

**What this does not establish:** that the MATLAB code runs, or that it is numerically correct. Static analysis has taken this as far as it can. Everything else needs MATLAB.

---

## Claims still not made

No safety certification · no guaranteed collision avoidance · no zero-accident claim · no real-world readiness · no real sensor performance · no detector capability · **no MATLAB test results** · **no simulation results** · **no toolbox availability**

---

**Bottom line: the Python side is verified and passing. The MATLAB side is structurally clean and has one genuine defect fixed, but remains entirely unexecuted.**
