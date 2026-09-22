"""Cross-check the MATLAB Frenet transforms against the Python port.

COMPONENT STATUS: REAL (cross-check tool)

WHY THIS EXISTS
---------------
The MATLAB functions ``projectPointOnPath`` and ``frenetToCartesian`` were
written independently of the Python Frenet model in
``existing_work/mathworks_scraper/python_port/frenet_motion_model.py``.

Two independent implementations of the same mathematics are worth far more
than one. If they agree to within numerical tolerance, the transform is very
probably correct. If they disagree, one of them is wrong -- and you know to
go looking, instead of trusting a single implementation that has never been
challenged.

This matters here specifically because the road-aligned (s, d) frame is the
coordinate system the whole IR-PSC planner works in. A sign error in the
lateral offset would make the planner steer *into* hazards rather than away
from them, and it would do so consistently enough to look like a design
choice rather than a bug.

HOW TO USE IT
-------------
1. In MATLAB, export reference data::

       setupPaths;
       exportFrenetReference('python/frenet_reference.csv');

   (See ``scripts/exportFrenetReference.m``.)

2. In Python, run this script::

       python python/crosscheck_frenet.py python/frenet_reference.csv

3. Read the report. Any row exceeding the tolerance is printed in full.

STATUS: NOT YET EXECUTED. MATLAB was not installed on the machine where this
was written, so no reference CSV has ever been produced and this comparison
has never been run. Run it on the destination laptop and report the real
result.

Requires: numpy. See ``python/requirements.txt``.
"""

from __future__ import annotations

import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

try:
    import numpy as np
except ImportError:  # pragma: no cover - dependency guard
    sys.exit("numpy is required. Install with: pip install -r python/requirements.txt")


# Tolerance for agreement between the two implementations. 1e-6 m is far
# below any physically meaningful distance here (a millimetre is already
# irrelevant at vehicle scale), so anything above this is a real algorithmic
# difference rather than floating-point noise.
TOLERANCE_M = 1e-6


@dataclass
class Row:
    """One reference case exported from MATLAB."""

    path_x: list[float]
    path_y: list[float]
    query_x: float
    query_y: float
    matlab_s: float
    matlab_d: float


def arc_length(path: np.ndarray) -> np.ndarray:
    """Cumulative arc length along a polyline. Mirrors pathArcLength.m."""
    if len(path) < 2:
        return np.zeros(len(path))
    seg = np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1))
    return np.concatenate([[0.0], np.cumsum(seg)])


def project_point(path: np.ndarray, pt: np.ndarray) -> tuple[float, float]:
    """Project a point onto a polyline, returning (s, d).

    Independent reimplementation of ``projectPointOnPath.m``. Written from
    the same mathematical definition, deliberately NOT translated line by
    line from the MATLAB -- a line-by-line translation would reproduce any
    bug faithfully and the comparison would prove nothing.

    Sign convention, which must match the MATLAB exactly:
        d > 0  =>  the point lies to the LEFT of the direction of travel.
    """
    s_cum = arc_length(path)
    best_d2 = math.inf
    best_s = 0.0
    best_side = 0.0

    for i in range(len(path) - 1):
        a = path[i]
        b = path[i + 1]
        ab = b - a
        len2 = float(ab @ ab)

        if len2 < 1e-12:
            t = 0.0
        else:
            t = float((pt - a) @ ab) / len2
            t = min(max(t, 0.0), 1.0)

        foot = a + t * ab
        d2 = float((pt - foot) @ (pt - foot))

        if d2 < best_d2:
            best_d2 = d2
            best_s = float(s_cum[i] + t * math.sqrt(len2))
            # z-component of cross(tangent, a->pt); positive means left.
            best_side = float(ab[0] * (pt[1] - a[1]) - ab[1] * (pt[0] - a[0]))

    d = math.sqrt(best_d2)
    if best_side < 0:
        d = -d
    elif best_side == 0:
        d = 0.0
    return best_s, d


def load_reference(path: Path) -> list[Row]:
    """Read the CSV exported by exportFrenetReference.m.

    Expected columns: n_path, then n_path pairs of px,py, then qx, qy,
    matlab_s, matlab_d.
    """
    rows: list[Row] = []
    with path.open(newline="", encoding="utf-8") as fh:
        reader = csv.reader(fh)
        header = next(reader, None)
        if header is None:
            raise ValueError(f"{path} is empty")

        for lineno, raw in enumerate(reader, start=2):
            if not raw:
                continue
            vals = [float(v) for v in raw]
            n = int(vals[0])
            expected = 1 + 2 * n + 4
            if len(vals) != expected:
                raise ValueError(
                    f"{path}:{lineno}: expected {expected} fields for "
                    f"n_path={n}, got {len(vals)}"
                )
            coords = vals[1 : 1 + 2 * n]
            rows.append(
                Row(
                    path_x=coords[0::2],
                    path_y=coords[1::2],
                    query_x=vals[1 + 2 * n],
                    query_y=vals[2 + 2 * n],
                    matlab_s=vals[3 + 2 * n],
                    matlab_d=vals[4 + 2 * n],
                )
            )
    return rows


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__)
        print("\nusage: python crosscheck_frenet.py <frenet_reference.csv>")
        return 2

    ref_path = Path(argv[1])
    if not ref_path.exists():
        print(f"Reference file not found: {ref_path}")
        print(
            "\nGenerate it first, in MATLAB:\n"
            "    setupPaths;\n"
            "    exportFrenetReference('python/frenet_reference.csv');"
        )
        return 1

    rows = load_reference(ref_path)
    if not rows:
        print("Reference file contains no rows.")
        return 1

    worst_s = 0.0
    worst_d = 0.0
    failures: list[tuple[int, float, float, float, float]] = []

    for idx, row in enumerate(rows, start=1):
        path = np.column_stack([row.path_x, row.path_y])
        pt = np.array([row.query_x, row.query_y])

        py_s, py_d = project_point(path, pt)

        err_s = abs(py_s - row.matlab_s)
        err_d = abs(py_d - row.matlab_d)
        worst_s = max(worst_s, err_s)
        worst_d = max(worst_d, err_d)

        if err_s > TOLERANCE_M or err_d > TOLERANCE_M:
            failures.append((idx, row.matlab_s, py_s, row.matlab_d, py_d))

    print("=" * 68)
    print("  FRENET CROSS-CHECK: MATLAB vs independent Python implementation")
    print("=" * 68)
    print(f"  cases compared : {len(rows)}")
    print(f"  tolerance      : {TOLERANCE_M:g} m")
    print(f"  worst |ds|     : {worst_s:.3e} m")
    print(f"  worst |dd|     : {worst_d:.3e} m")
    print("-" * 68)

    if failures:
        print(f"  RESULT: {len(failures)} DISAGREEMENT(S)\n")
        for idx, m_s, p_s, m_d, p_d in failures[:20]:
            print(
                f"    row {idx:4d}:  s MATLAB={m_s:+.9f} Python={p_s:+.9f}"
                f"  |  d MATLAB={m_d:+.9f} Python={p_d:+.9f}"
            )
        if len(failures) > 20:
            print(f"    ... and {len(failures) - 20} more")
        print(
            "\n  One of the two implementations is wrong. A sign difference in d\n"
            "  is the most likely cause and the most dangerous: it would make the\n"
            "  planner deform toward hazards instead of away from them.\n"
        )
        print("=" * 68)
        return 1

    print("  RESULT: AGREEMENT within tolerance")
    print(
        "\n  Both implementations agree. This is evidence the road-aligned\n"
        "  transform is correct -- not proof, but far stronger evidence than\n"
        "  a single unchallenged implementation."
    )
    print("=" * 68)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
