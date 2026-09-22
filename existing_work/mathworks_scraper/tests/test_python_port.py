"""Tests for the hand-written Python port of the documented motion model.

These validate the port against the *mathematics stated on the page* rather
than against its own output: the transition matrix entries, the inclusive
range semantics, and the qualitative claims the example makes about
road-integrated prediction versus a constant-velocity model.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")

PORT_DIR = Path(__file__).resolve().parent.parent / "python_port"
if str(PORT_DIR) not in sys.path:
    sys.path.insert(0, str(PORT_DIR))

from frenet_motion_model import (  # noqa: E402
    FrenetMotionModel,
    ReferencePath,
    constant_velocity_transition,
    predict_trajectory,
)


@pytest.fixture
def straight_path() -> ReferencePath:
    return ReferencePath(np.column_stack([np.linspace(0, 500, 51), np.zeros(51)]))


@pytest.fixture
def curved_path() -> ReferencePath:
    angles = np.linspace(0, math.radians(60), 300)
    return ReferencePath(
        np.column_stack([200 * np.sin(angles), 200 * (1 - np.cos(angles))])
    )


class TestReferencePath:
    def test_length_of_straight_path(self, straight_path):
        assert straight_path.length == pytest.approx(500.0)

    def test_frenet2global_on_centreline(self, straight_path):
        x, y = straight_path.frenet2global(100.0, 0.0)
        assert (x, y) == pytest.approx((100.0, 0.0))

    def test_positive_d_is_to_the_left(self, straight_path):
        _, y = straight_path.frenet2global(100.0, 3.5)
        assert y == pytest.approx(3.5)

    def test_round_trip_conversion(self, curved_path):
        for s, d in [(10.0, 0.0), (75.0, -3.5), (150.0, 2.0)]:
            x, y = curved_path.frenet2global(s, d)
            s_back, d_back = curved_path.global2frenet(x, y)
            assert s_back == pytest.approx(s, abs=0.5)
            assert d_back == pytest.approx(d, abs=0.05)

    def test_arc_length_matches_analytic_circle(self, curved_path):
        assert curved_path.length == pytest.approx(200 * math.radians(60), rel=1e-3)

    def test_s_is_clamped_to_the_path(self, straight_path):
        x, _ = straight_path.frenet2global(10_000.0, 0.0)
        assert x == pytest.approx(500.0)

    def test_rejects_too_few_waypoints(self):
        with pytest.raises(ValueError):
            ReferencePath(np.array([[0.0, 0.0]]))

    def test_rejects_bad_shape(self):
        with pytest.raises(ValueError):
            ReferencePath(np.zeros((5, 3)))

    def test_rejects_duplicate_waypoints(self):
        with pytest.raises(ValueError):
            ReferencePath(np.array([[0.0, 0.0], [0.0, 0.0], [1.0, 0.0]]))


class TestTransitionMatrix:
    """# From: Object State Transition and Measurement Modeling"""

    def test_matches_published_matrix(self):
        model = FrenetMotionModel(tau=3.0)
        dt, tau = 0.5, 3.0
        f = model.transition_matrix(dt)
        expected = np.array(
            [
                [1, dt, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, tau * (1 - math.exp(-dt / tau))],
                [0, 0, 0, math.exp(-dt / tau)],
            ]
        )
        assert np.allclose(f, expected)

    def test_zero_dt_is_identity(self):
        assert np.allclose(FrenetMotionModel().transition_matrix(0.0), np.eye(4))

    def test_longitudinal_channel_is_constant_speed(self):
        model = FrenetMotionModel(tau=3.0)
        state = model.predict([0.0, 25.0, 0.0, 0.0], 2.0)
        assert state[0] == pytest.approx(50.0)
        assert state[1] == pytest.approx(25.0)  # speed unchanged

    def test_lateral_velocity_decays(self):
        model = FrenetMotionModel(tau=3.0)
        state = model.predict([0, 25, 0, 1.0], 3.0)
        assert state[3] == pytest.approx(math.exp(-1.0))
        assert state[3] < 1.0

    def test_lateral_offset_converges_to_a_finite_limit(self):
        """A lane change settles instead of drifting sideways forever."""
        model = FrenetMotionModel(tau=3.0)
        state = np.array([0.0, 25.0, -1.8, 1.2])
        for _ in range(200):
            state = model.predict(state, 0.5)
        limit = -1.8 + 3.0 * 1.2  # d0 + tau * d_dot0
        assert state[2] == pytest.approx(limit, abs=1e-6)
        assert state[3] == pytest.approx(0.0, abs=1e-9)

    def test_large_tau_approaches_constant_velocity(self):
        model = FrenetMotionModel(tau=1e6)
        assert np.allclose(
            model.transition_matrix(0.5), constant_velocity_transition(0.5), atol=1e-5
        )

    def test_rejects_non_positive_tau(self):
        with pytest.raises(ValueError):
            FrenetMotionModel(tau=0)

    def test_rejects_negative_dt(self):
        with pytest.raises(ValueError):
            FrenetMotionModel().transition_matrix(-1.0)


class TestProcessNoise:
    def test_shape_and_symmetry(self):
        q = FrenetMotionModel().process_noise(0.5)
        assert q.shape == (4, 4)
        assert np.allclose(q, q.T)

    def test_positive_semidefinite(self):
        q = FrenetMotionModel(sigma_s=1.0, sigma_d=0.5).process_noise(0.5)
        assert np.all(np.linalg.eigvalsh(q) >= -1e-12)

    def test_matches_g_qa_gt(self):
        model = FrenetMotionModel(sigma_s=2.0, sigma_d=0.5)
        dt = 0.5
        g = model.noise_gain(dt)
        expected = g @ np.diag([4.0, 0.25]) @ g.T
        assert np.allclose(model.process_noise(dt), expected)

    def test_uncertainty_grows_with_time(self):
        model = FrenetMotionModel()
        cov = np.diag([1.0, 1.0, 0.5, 0.5])
        first = cov[0, 0]
        for _ in range(5):
            cov = model.predict_covariance(cov, 1.0)
        assert cov[0, 0] > first

    def test_rejects_negative_sigma(self):
        with pytest.raises(ValueError):
            FrenetMotionModel(sigma_s=-1.0)


class TestTrajectoryPrediction:
    def test_step_count_matches_horizon(self, straight_path):
        """# From: Run Simulation -- tHorizon = 5, deltaT = 0.5 -> 10 steps."""
        traj = predict_trajectory(
            FrenetMotionModel(), [0, 25, 0, 0], straight_path, horizon=5.0, dt=0.5
        )
        assert len(traj) == 11  # t=0 plus 10 predicted steps
        assert traj[-1]["time"] == pytest.approx(5.0)

    def test_positions_are_monotonic_along_the_path(self, straight_path):
        traj = predict_trajectory(
            FrenetMotionModel(), [0, 25, 0, 0], straight_path, 5.0, 0.5
        )
        s_values = [p["s"] for p in traj]
        assert s_values == sorted(s_values)

    def test_global_coordinates_present(self, curved_path):
        traj = predict_trajectory(
            FrenetMotionModel(), [0, 20, 0, 0], curved_path, 3.0, 1.0
        )
        assert all("x" in p and "y" in p for p in traj)

    def test_road_integrated_prediction_stays_in_lane(self, curved_path):
        """The page's core claim: the Frenet model tracks the curve."""
        traj = predict_trajectory(
            FrenetMotionModel(), [0.0, 25.0, 0.0, 0.0], curved_path, 5.0, 0.5
        )
        for point in traj:
            _, lateral = curved_path.global2frenet(point["x"], point["y"])
            assert abs(lateral) < 0.1  # never leaves the lane centre

    def test_constant_velocity_leaves_the_lane_on_a_curve(self, curved_path):
        """The failure mode the example describes as a false collision."""
        x0, y0 = curved_path.frenet2global(0.0, 0.0)
        heading = curved_path.heading_at(0.0)
        x = x0 + 25.0 * math.cos(heading) * 5.0
        y = y0 + 25.0 * math.sin(heading) * 5.0
        _, lateral = curved_path.global2frenet(x, y)
        assert abs(lateral) > 3.5  # more than a lane width off

    def test_rejects_bad_horizon(self, straight_path):
        with pytest.raises(ValueError):
            predict_trajectory(
                FrenetMotionModel(), [0, 1, 0, 0], straight_path, horizon=0
            )

    def test_rejects_bad_dt(self, straight_path):
        with pytest.raises(ValueError):
            predict_trajectory(FrenetMotionModel(), [0, 1, 0, 0], straight_path, 5.0, 0)

    def test_rejects_wrong_state_size(self, straight_path):
        with pytest.raises(ValueError):
            predict_trajectory(FrenetMotionModel(), [0, 1, 0], straight_path)


def test_demo_runs_without_error(capsys):
    import frenet_motion_model

    frenet_motion_model.main()
    out = capsys.readouterr().out
    assert "Lane change prediction" in out
    assert "divergence after 5 s" in out
