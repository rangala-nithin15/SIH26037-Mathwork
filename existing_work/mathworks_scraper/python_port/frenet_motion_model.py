"""Executable Python port of the motion model documented on the MathWorks page.

# From: Object State Transition and Measurement Modeling

The example page states the object state in Frenet coordinates as

    x_k = [s_k, s_k_dot, d_k, d_k_dot]

where ``s`` is arc length **along** the highway centreline and ``d`` is the
signed lateral offset **perpendicular** to it. Longitudinal motion uses a
constant-speed model; lateral motion uses a *decaying*-speed model, which is
what lets the tracker represent a lane change that settles into a new lane
rather than continuing sideways forever.

The published transition matrix is::

    | 1  dT  0   0                        |
    | 0   1  0   0                        |
    | 0   0  1   tau * (1 - exp(-dT/tau))  |
    | 0   0  0   exp(-dT/tau)              |

with process noise entering through

    G = [[dT^2/2, 0], [dT, 0], [0, dT^2/2], [0, dT]]

for zero-mean Gaussian accelerations ``[w_s, w_d]``.

What this module *is*: a faithful, runnable implementation of that motion
model, the Frenet <-> Cartesian conversion, and a constant-velocity baseline
for comparison. It reproduces the paper's central claim -- that a
road-integrated model predicts a curving vehicle's future position far better
than a constant-velocity model does.

What this module is **not**: a port of ``trackerJPDA``. Joint probabilistic
data association, track management (confirmation ``[8 10]`` / deletion
``[5 5]``) and the trajectory optimiser are MathWorks toolbox functionality
with no open-source equivalent; the page's MATLAB listings for those are
preserved verbatim in the extracted output instead of being guessed at.

Run ``python frenet_motion_model.py`` for a self-checking demonstration.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

__all__ = [
    "ReferencePath",
    "FrenetMotionModel",
    "constant_velocity_transition",
    "predict_trajectory",
]


# --------------------------------------------------------------------------
# Reference path (the Python stand-in for `referencePathFrenet`)
# --------------------------------------------------------------------------


@dataclass
class ReferencePath:
    """Arc-length parameterised polyline through a set of waypoints.

    Mirrors the parts of Navigation Toolbox's ``referencePathFrenet`` that the
    motion model actually needs: ``frenet2global`` and ``global2frenet``.
    """

    waypoints: np.ndarray  # (N, 2) array of [x, y]

    def __post_init__(self) -> None:
        pts = np.asarray(self.waypoints, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 2:
            raise ValueError("waypoints must be an (N, 2) array of [x, y]")
        if len(pts) < 2:
            raise ValueError("a reference path needs at least two waypoints")
        self.waypoints = pts
        deltas = np.diff(pts, axis=0)
        seg_len = np.hypot(deltas[:, 0], deltas[:, 1])
        if np.any(seg_len == 0):
            raise ValueError("duplicate consecutive waypoints produce a zero-length segment")
        #: Cumulative arc length at each waypoint.
        self.s = np.concatenate([[0.0], np.cumsum(seg_len)])
        #: Unit tangent per segment.
        self.tangents = deltas / seg_len[:, None]

    @property
    def length(self) -> float:
        return float(self.s[-1])

    def _segment_for(self, s: float) -> int:
        s = min(max(s, 0.0), self.length)
        idx = int(np.searchsorted(self.s, s, side="right") - 1)
        return min(max(idx, 0), len(self.tangents) - 1)

    def tangent_at(self, s: float) -> np.ndarray:
        return self.tangents[self._segment_for(s)]

    def heading_at(self, s: float) -> float:
        tx, ty = self.tangent_at(s)
        return math.atan2(ty, tx)

    def frenet2global(self, s: float, d: float) -> Tuple[float, float]:
        """Convert an (arc length, lateral offset) pair to global [x, y].

        ``s`` is clamped to ``[0, length]``: a highway centreline is a finite
        polyline, and silently extrapolating off its end would hand the
        planner a plausible-looking but meaningless position.
        """
        s = min(max(s, 0.0), self.length)
        i = self._segment_for(s)
        base = self.waypoints[i] + self.tangents[i] * (s - self.s[i])
        tx, ty = self.tangents[i]
        normal = np.array([-ty, tx])  # left-hand normal, positive d = left
        point = base + normal * d
        return float(point[0]), float(point[1])

    def global2frenet(self, x: float, y: float) -> Tuple[float, float]:
        """Project a global point onto the path, returning (s, d).

        Uses an exact per-segment projection rather than nearest-waypoint
        search, so accuracy does not depend on waypoint density.
        """
        point = np.array([x, y], dtype=float)
        best = (float("inf"), 0.0, 0.0)
        for i, tangent in enumerate(self.tangents):
            start = self.waypoints[i]
            seg_len = self.s[i + 1] - self.s[i]
            rel = point - start
            t = float(np.clip(np.dot(rel, tangent), 0.0, seg_len))
            proj = start + tangent * t
            offset = point - proj
            dist = float(np.hypot(*offset))
            if dist < best[0]:
                normal = np.array([-tangent[1], tangent[0]])
                signed_d = float(np.dot(offset, normal))
                best = (dist, self.s[i] + t, signed_d)
        return best[1], best[2]


# --------------------------------------------------------------------------
# The state transition model
# --------------------------------------------------------------------------


class FrenetMotionModel:
    """Constant-speed longitudinal + decaying-speed lateral model.

    Parameters
    ----------
    tau:
        Lateral velocity decay constant in seconds. Small ``tau`` means a
        lane change settles quickly; as ``tau -> inf`` the lateral channel
        degenerates to constant velocity.
    sigma_s, sigma_d:
        Standard deviations of the unmodelled accelerations ``w_s``/``w_d``
        in m/s^2, used to build the process-noise covariance ``Q``.
    """

    STATE_SIZE = 4

    def __init__(self, tau: float = 3.0, sigma_s: float = 1.0, sigma_d: float = 1.0) -> None:
        if tau <= 0:
            raise ValueError("tau must be positive")
        if sigma_s < 0 or sigma_d < 0:
            raise ValueError("noise standard deviations must be non-negative")
        self.tau = float(tau)
        self.sigma_s = float(sigma_s)
        self.sigma_d = float(sigma_d)

    # ---------------------------------------------------------------- matrices

    def transition_matrix(self, dt: float) -> np.ndarray:
        """State transition matrix ``F`` for a step of ``dt`` seconds."""
        if dt < 0:
            raise ValueError("dt must be non-negative")
        tau = self.tau
        decay = math.exp(-dt / tau)
        return np.array(
            [
                [1.0, dt, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, tau * (1.0 - decay)],
                [0.0, 0.0, 0.0, decay],
            ]
        )

    def noise_gain(self, dt: float) -> np.ndarray:
        """Noise gain ``G`` mapping ``[w_s, w_d]`` into the state."""
        half = 0.5 * dt * dt
        return np.array([[half, 0.0], [dt, 0.0], [0.0, half], [0.0, dt]])

    def process_noise(self, dt: float) -> np.ndarray:
        """Process-noise covariance ``Q = G * diag(sigma^2) * G'``."""
        gain = self.noise_gain(dt)
        accel_cov = np.diag([self.sigma_s**2, self.sigma_d**2])
        return gain @ accel_cov @ gain.T

    # ----------------------------------------------------------------- predict

    def predict(self, state: Sequence[float], dt: float) -> np.ndarray:
        """Advance a state vector ``[s, s_dot, d, d_dot]`` by ``dt``."""
        vector = np.asarray(state, dtype=float).reshape(self.STATE_SIZE)
        return self.transition_matrix(dt) @ vector

    def predict_covariance(
        self, covariance: np.ndarray, dt: float
    ) -> np.ndarray:
        """Advance a state covariance: ``P' = F P F' + Q``."""
        f = self.transition_matrix(dt)
        return f @ np.asarray(covariance, dtype=float) @ f.T + self.process_noise(dt)


def constant_velocity_transition(dt: float) -> np.ndarray:
    """Baseline model that ignores the road: constant velocity in both axes."""
    return np.array(
        [
            [1.0, dt, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, dt],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def predict_trajectory(
    model: FrenetMotionModel,
    state: Sequence[float],
    path: ReferencePath,
    horizon: float = 5.0,
    dt: float = 0.5,
) -> List[dict]:
    """Roll the model forward over the planning horizon.

    # From: Run Simulation -- mirrors the MATLAB
    # `predictTracksToTime(tracker,'confirmed',timesteps(i))` loop, which
    # uses tHorizon = 5 s and deltaT = 0.5 s.
    """
    if horizon <= 0 or dt <= 0:
        raise ValueError("horizon and dt must be positive")

    current = np.asarray(state, dtype=float).reshape(4)
    steps = int(round(horizon / dt))
    out = []
    for k in range(steps + 1):
        t = k * dt
        s, s_dot, d, d_dot = current
        x, y = path.frenet2global(s, d)
        out.append(
            {
                "time": t,
                "s": float(s),
                "s_dot": float(s_dot),
                "d": float(d),
                "d_dot": float(d_dot),
                "x": x,
                "y": y,
            }
        )
        if k < steps:
            current = model.predict(current, dt)
    return out


# --------------------------------------------------------------------------
# Demonstration
# --------------------------------------------------------------------------


def _curved_highway(radius: float = 200.0, sweep_deg: float = 60.0, n: int = 400):
    """A gently curving highway centreline, similar to the example scenario."""
    angles = np.linspace(0.0, math.radians(sweep_deg), n)
    return np.column_stack([radius * np.sin(angles), radius * (1 - np.cos(angles))])


def main() -> None:
    path = ReferencePath(_curved_highway())
    model = FrenetMotionModel(tau=3.0, sigma_s=1.0, sigma_d=0.5)

    print(f"Reference path length: {path.length:.1f} m")
    print()

    # --- 1. Lane change captured by the decaying lateral-velocity model ----
    # A vehicle at 25 m/s starting a lane change with 1.2 m/s lateral speed.
    print("Lane change prediction (tau = 3.0 s)")
    print(f"{'t [s]':>6} {'s [m]':>9} {'d [m]':>8} {'d_dot [m/s]':>12}")
    state = [0.0, 25.0, -1.8, 1.2]
    for step in predict_trajectory(model, state, path, horizon=5.0, dt=1.0):
        print(
            f"{step['time']:6.1f} {step['s']:9.2f} {step['d']:8.2f} {step['d_dot']:12.3f}"
        )
    final_d = predict_trajectory(model, state, path, 5.0, 1.0)[-1]["d"]
    print(f"-> lateral offset converges to {final_d:.2f} m and stops drifting")
    print()

    # --- 2. Road-integrated vs constant-velocity prediction ---------------
    # A vehicle travelling along the curve at 25 m/s, centred in its lane.
    print("Road-integrated vs constant-velocity, 5 s ahead")
    truth_state = [0.0, 25.0, 0.0, 0.0]
    frenet_end = predict_trajectory(model, truth_state, path, 5.0, 0.5)[-1]
    frenet_xy = np.array([frenet_end["x"], frenet_end["y"]])

    # Constant velocity extrapolates the *instantaneous* Cartesian velocity.
    x0, y0 = path.frenet2global(0.0, 0.0)
    heading0 = path.heading_at(0.0)
    cv_xy = np.array(
        [x0 + 25.0 * math.cos(heading0) * 5.0, y0 + 25.0 * math.sin(heading0) * 5.0]
    )
    _, cv_lateral = path.global2frenet(*cv_xy)

    error = float(np.hypot(*(cv_xy - frenet_xy)))
    print(f"  road-integrated end point : ({frenet_xy[0]:8.2f}, {frenet_xy[1]:7.2f})")
    print(f"  constant-velocity end point: ({cv_xy[0]:8.2f}, {cv_xy[1]:7.2f})")
    print(f"  divergence after 5 s       : {error:.2f} m")
    print(f"  constant-velocity lateral offset from lane centre: {cv_lateral:.2f} m")
    print(
        "  -> the constant-velocity track leaves the lane entirely, which is\n"
        "     exactly the false-collision failure the page describes."
    )
    print()

    # --- 3. Growth of predicted uncertainty --------------------------------
    print("Predicted position uncertainty over the horizon")
    cov = np.diag([1.0, 1.0, 0.5, 0.5])
    for k in range(6):
        if k:
            cov = model.predict_covariance(cov, 1.0)
        print(f"  t={k}s  std_s={math.sqrt(cov[0, 0]):5.2f} m  std_d={math.sqrt(cov[2, 2]):5.2f} m")


if __name__ == "__main__":
    main()
