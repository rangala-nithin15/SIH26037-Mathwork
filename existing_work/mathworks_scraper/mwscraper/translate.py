"""Conservative MATLAB -> Python translation.

Design stance
-------------
Automatic MATLAB-to-Python translation is *not* generally sound: MATLAB is
1-indexed with inclusive ranges, ``a(i)`` is ambiguous between indexing and a
function call, ``end`` is context-dependent, and the toolbox objects used on
this page (``trackerJPDA``, ``referencePathFrenet``, ``dynamicCapsuleList``)
have no Python counterpart at all.

So this module refuses to guess. It emits a Python equivalent **only** when
every line of a snippet falls into a small, provably safe subset:

* comments (``%`` -> ``#``)
* blank lines
* scalar numeric assignments (``tHorizon = 5;``)
* simple ``a:b`` / ``a:step:b`` ranges over scalars, mapped to ``numpy.arange``
  with the inclusive endpoint corrected

Anything else yields ``(None, reason)``, and the reason is carried into the
output so a human reader knows the block was deliberately left untranslated
rather than missed. Hand-written, genuinely executable Python for the core
algorithm lives in ``python_port/frenet_motion_model.py``.
"""

from __future__ import annotations

import re
from typing import List, Optional, Tuple

# `x = 5;`  /  `x = 5.0`  /  `x = -0.25;  % comment`
_SCALAR_ASSIGN = re.compile(
    r"^\s*([A-Za-z_]\w*)\s*=\s*(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\s*;?\s*(%.*)?$"
)

# `tSteps = deltaT:deltaT:tHorizon;`  /  `v = 1:10;`
_RANGE_ASSIGN = re.compile(
    r"^\s*([A-Za-z_]\w*)\s*=\s*([A-Za-z_]\w*|-?[\d.]+)\s*:\s*"
    r"(?:([A-Za-z_]\w*|-?[\d.]+)\s*:\s*)?([A-Za-z_]\w*|-?[\d.]+)\s*;?\s*(%.*)?$"
)

_COMMENT_ONLY = re.compile(r"^\s*%(.*)$")


def _convert_comment(text: str) -> str:
    return "#" + text.rstrip()


def translate_matlab_snippet(code: str) -> Tuple[Optional[str], Optional[str]]:
    """Translate ``code`` to Python, or explain why it was not translated.

    Returns
    -------
    (python_code, note)
        ``python_code`` is ``None`` when translation would require guessing.
        ``note`` always carries a human-readable explanation.
    """
    if not code or not code.strip():
        return None, "Empty snippet."

    lines = code.split("\n")
    out: List[str] = []
    needs_numpy = False
    blockers: List[str] = []

    for raw in lines:
        line = raw.rstrip()

        if not line.strip():
            out.append("")
            continue

        comment = _COMMENT_ONLY.match(line)
        if comment:
            out.append(_convert_comment(comment.group(1)))
            continue

        scalar = _SCALAR_ASSIGN.match(line)
        if scalar:
            name, value, trailing = scalar.group(1), scalar.group(2), scalar.group(3)
            rendered = f"{name} = {value}"
            if trailing:
                rendered += "  " + _convert_comment(trailing[1:])
            out.append(rendered)
            continue

        rng = _RANGE_ASSIGN.match(line)
        if rng:
            name, start, step, stop, trailing = rng.groups()
            step = step or "1"
            needs_numpy = True
            # MATLAB ranges include the endpoint; np.arange does not, so the
            # stop is nudged by half a step.
            rendered = (
                f"{name} = np.arange({start}, {stop} + {step} / 2, {step})"
            )
            if trailing:
                rendered += "  " + _convert_comment(trailing[1:])
            out.append(rendered)
            continue

        blockers.append(line.strip()[:60])

    if blockers:
        sample = "; ".join(blockers[:3])
        return None, (
            "Not auto-translated: the snippet uses MATLAB constructs with no "
            "safe mechanical Python equivalent (e.g. "
            f"{sample}). Toolbox objects such as trackerJPDA, "
            "referencePathFrenet and dynamicCapsuleList have no Python "
            "counterpart. See python_port/frenet_motion_model.py for a "
            "hand-written implementation of the underlying motion model."
        )

    if needs_numpy:
        out.insert(0, "import numpy as np")

    python_code = "\n".join(out).strip("\n")
    if not python_code:
        return None, "Snippet contained no translatable statements."

    return python_code, (
        "Auto-translated from MATLAB using the safe subset "
        "(comments, scalar assignments, ranges). Verify before use."
    )
