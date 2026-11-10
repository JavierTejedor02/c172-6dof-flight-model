"""Local-model mechanics, cusp behavior, modal invariance and pulse comparisons."""
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from c172sim import Aircraft, Controls
from c172sim.dynamics import rhs
from c172sim.experiments import checked_trim_state, pulse_segments
from c172sim.linearization import (
    LinearModel, STATE_SCALES, beta_drag_direction, controls_from_vector,
    input_vector, linearize_aircraft, matrix_refinement_check, mode_analysis,
    simulate_perturbations,
)
from run_linear_analysis import run_analysis, save_analysis


class LinearizationChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.aircraft = Aircraft()
        filename = Path(__file__).resolve().parents[1] / "results" / "longitudinal" / "trim_and_run.json"
        cls.state, cls.controls = checked_trim_state(cls.aircraft, filename)
        cls.model = linearize_aircraft(cls.aircraft, cls.state, cls.controls, 0.5)

    def test_beta_drag_split_matches_the_original_force_and_moment_model(self):
        no_beta_drag = Aircraft()
        no_beta_drag.c["CDbeta"] = 0.0
        for beta in (-0.01, 0.01):
            state = self.state.copy()
            state[2:6] = [beta, 0.01, -0.02, 0.03]
            difference = rhs(0, state, self.aircraft, self.controls) - rhs(0, state, no_beta_drag, self.controls)
            np.testing.assert_allclose(difference, abs(beta) * beta_drag_direction(self.aircraft, state), atol=1e-11, rtol=0)

    def test_one_sided_beta_derivatives_disagree_by_the_predicted_cusp(self):
        diagnostics = self.model.diagnostics
        measured = np.array(diagnostics["beta_derivative_gap"]) / 2
        self.assertLess(self.model.cusp[0], -1)
        self.assertGreater(abs(self.model.cusp[4]), 0.1)
        np.testing.assert_allclose(measured, self.model.cusp, atol=2e-5, rtol=0)
        self.assertTrue(diagnostics["zero_beta_cusp"])

    def test_directional_first_order_model_has_second_order_local_remainder(self):
        dx = np.array([1, 0.02, 0.01, 0.01, 0.02, -0.01, 0.02, -0.02, 0, 0, 0, 0])
        du = np.array([0.01, -0.005, 0.003, 10])
        baseline = rhs(0, self.state, self.aircraft, self.controls)
        for sign in (-1, 1):
            errors = []
            for epsilon in (0.02, 0.01):
                nonlinear = rhs(0, self.state + sign * epsilon * dx, self.aircraft,
                                controls_from_vector(input_vector(self.controls) + sign * epsilon * du)) - baseline
                directional = self.model.derivative(sign * epsilon * dx, sign * epsilon * du, True)
                central = self.model.derivative(sign * epsilon * dx, sign * epsilon * du, False)
                error = np.linalg.norm((nonlinear - directional)[:8] / STATE_SCALES[:8])
                self.assertLess(error, np.linalg.norm((nonlinear - central)[:8] / STATE_SCALES[:8]))
                errors.append(error)
            self.assertGreater(errors[0] / errors[1], 3.5)
            self.assertLess(errors[0] / errors[1], 4.5)

    def test_differentiation_refinement_and_physical_data_are_preserved(self):
        before = json.dumps(self.aircraft.data, sort_keys=True)
        coarse = linearize_aircraft(self.aircraft, self.state, self.controls)
        self.assertTrue(matrix_refinement_check(coarse, self.model)["passed"])
        self.assertEqual(self.model.A.shape, (12, 12))
        self.assertEqual(self.model.B.shape, (12, 4))
        self.assertEqual(before, json.dumps(self.aircraft.data, sort_keys=True))

    def test_moving_trim_and_cyclic_coordinates_are_distinguished(self):
        future = self.model.nominal_state(1)
        np.testing.assert_array_equal(future[:9], self.model.state[:9])
        self.assertGreater(future[9] - self.model.state[9], 45)
        np.testing.assert_allclose(self.model.A[:8, 8:], 0, atol=1e-12)
        full = np.linalg.eigvals(self.model.A)
        dynamic = np.linalg.eigvals(self.model.A[:8, :8])
        self.assertGreaterEqual(np.count_nonzero(np.abs(full) < 1e-7), 4)
        self.assertTrue(np.all(np.abs(dynamic) > 1e-3))

    def test_zero_input_perturbation_stays_zero(self):
        segments = pulse_segments(self.controls, "elevator", 0, duration=4)
        for directional in (False, True):
            result = simulate_perturbations(self.model, segments, 0.02, directional)
            np.testing.assert_array_equal(result.states, np.zeros_like(result.states))
            self.assertTrue(np.all(np.diff(result.time) > 0))
            self.assertEqual(result.nfev, 4 * (len(result.time) - 1))

    def test_other_lookup_corners_are_rejected_without_changing_trim_loads(self):
        aircraft = Aircraft()
        alpha = self.state[1]
        value = aircraft.table("CL_static", alpha)
        aircraft.tables["CL_static"] = np.array([[alpha - 0.01, value - 0.04],
                                                 [alpha, value],
                                                 [alpha + 0.01, value + 0.03]])
        np.testing.assert_allclose(rhs(0, self.state, aircraft, self.controls),
                                   rhs(0, self.state, self.aircraft, self.controls), atol=1e-10)
        with self.assertRaisesRegex(ValueError, "slope corner"):
            linearize_aircraft(aircraft, self.state, self.controls)

    def test_modal_frequency_and_participation_are_unit_invariant(self):
        A = np.zeros((12, 12))
        A[np.ix_([0, 1], [0, 1])] = [[-0.1, 1], [-4, -0.1]]
        A[np.ix_([4, 7], [4, 7])] = [[-1, 3], [-3, -1]]
        A[np.ix_([2, 5], [2, 5])] = [[-0.2, 2], [-2, -0.2]]
        A[3, 3], A[6, 6] = -5, -0.02
        synthetic = LinearModel(A, np.zeros((12, 4)), np.zeros(12), self.state, Controls(), {})
        modes = mode_analysis(synthetic)["modes"]
        phugoid = next(mode for mode in modes if mode["candidate_label"] == "phugoid_candidate")
        self.assertAlmostEqual(phugoid["period_s"], np.pi)
        self.assertAlmostEqual(phugoid["damping_ratio"], 0.1 / np.sqrt(4.01))
        scales = np.array([2, 3, 5, 7, 11, 13, 17, 19, 1, 1, 1, 1])
        transformed = replace(synthetic, A=A * scales[:, None] / scales[None, :])
        changed = mode_analysis(transformed)["modes"]
        for first, second in zip(modes, changed):
            self.assertAlmostEqual(first["eigenvalue_real_s_inv"], second["eigenvalue_real_s_inv"])
            np.testing.assert_allclose(list(first["participation"].values()),
                                       list(second["participation"].values()), atol=1e-12)

    def test_pulse_comparison_checks_all_surfaces_and_both_signs(self):
        model, report, curves = run_analysis(self.aircraft, self.state, self.controls,
                                         amplitudes=[0.25], duration=4, step_size=0.02, check_step=True)
        self.assertEqual(len(report["cases"]), 6)
        self.assertTrue(report["integration_refinement"]["passed"])
        for case in report["cases"]:
            self.assertTrue(np.isfinite(case["directional"]["max_dynamic_scaled_error"]))
            if case["surface"] in ("aileron", "rudder"):
                self.assertLess(case["directional"]["max_dynamic_scaled_error"],
                                case["central_lti"]["max_dynamic_scaled_error"])
            np.testing.assert_array_equal(curves[case["case"]]["nonlinear_delta"][0], np.zeros(12))
        with tempfile.TemporaryDirectory() as temporary:
            save_analysis(temporary, model, report, curves)
            saved = json.loads((Path(temporary) / "linearization.json").read_text(encoding="utf-8"))
            np.testing.assert_array_equal(saved["A"], model.A)
            with np.load(Path(temporary) / "perturbation_trajectories.npz", allow_pickle=False) as archive:
                self.assertEqual(len([key for key in archive if key.endswith("__time")]), 6)


if __name__ == "__main__":
    unittest.main()
