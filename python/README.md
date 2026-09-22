# Python components

**Component status: SUPPORTING / CROSS-CHECK — not the deliverable**

---

## What Python is and is not, in this project

The deliverable is the **MATLAB/Simulink** system. Python plays two supporting roles and no others:

1. **Independent numerical cross-check.** The Frenet motion model was ported to Python before the MATLAB implementation existed. Comparing the two catches errors that trusting a single implementation never would — if both agree to 1e-9, the maths is very probably right; if they disagree, one of them is wrong and you know to look.

2. **Prototyping ground.** Detector experiments, data exploration, quick numerical checks. Nothing here is on the critical path.

**Nothing in Python is presented as the final implementation of anything.** If a Python file and a MATLAB file appear to do the same job, the MATLAB one is the deliverable and the Python one is the check.

## Layout

| Path | What it is |
|---|---|
| `python/crosscheck_frenet.py` | Compares MATLAB-exported trajectory data against the Python Frenet model |
| `python/requirements.txt` | Dependencies for the cross-check |
| `../existing_work/mathworks_scraper/` | The preserved prior work — see below |

## The preserved prior work

`../existing_work/mathworks_scraper/` is a **complete copy** of the Python package written before this project began. It was copied in unchanged, and the original outside this project was not modified.

It contains:

- `mwscraper/` — an 11-module scraper package
- `python_port/frenet_motion_model.py` — **the valuable part**: a runnable implementation of the Frenet-coordinate motion model with state `[s, ṡ, d, ḋ]`, constant longitudinal speed, decaying lateral speed, the process-noise matrix, Frenet↔Cartesian conversion, and a constant-velocity baseline
- `tests/` — 6 test files including `fixtures/frenet_page.html`, which lets the whole suite run with no network access
- `output/code/` — 10 `.m` listings extracted from MathWorks documentation

### About those 10 `.m` files — read this before using them

They are **third-party MathWorks documentation excerpts**, kept as reference material. They have never been executed, they require toolboxes this project does not assume, and they must **not** be copied wholesale into the deliverable or presented as our own work. Respect attribution in any SIH submission.

### About the Frenet port — the honest scope note

The MathWorks example it was ported from assumes a **structured highway with lane markings**. This project's premise is the opposite.

We reuse the **mathematics** — the coordinate transform, the motion model, the noise structure. What we replace is where the reference path comes from: in IR-PSC it is the drivable-space corridor centreline, never a detected lane. That substitution is part of the contribution and is documented as such. **Frenet coordinates themselves are not claimed as our innovation** — they are a standard technique used as a supporting tool.

## Running it

```bash
cd existing_work/mathworks_scraper
pip install -r requirements.txt
python -m pytest
```

The README in that folder states 174 tests, fully offline. **That is the README's claim, not a measured result** — the suite has not been executed during this project. Run it and report the real number.

```bash
python python_port/frenet_motion_model.py
```

## Python version

Use **Python 3.11 or 3.12**. Avoid 3.14: it is likely newer than any current MATLAB release accepts, which matters if a MATLAB↔Python bridge is ever needed. Two interpreters side by side is normal and safe.
