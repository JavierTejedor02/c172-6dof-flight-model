"""Analytic checks of the standalone Euler and RK4 formulas (NumPy only)."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

# Load the standalone file so these mechanics-free checks do not require SciPy
# through the package's trim/simulation imports.
spec = importlib.util.spec_from_file_location(
    "educational_integrators", Path(__file__).resolve().parents[1] / "c172sim" / "integrators.py")
integrators = importlib.util.module_from_spec(spec)
spec.loader.exec_module(integrators)


def integrate(step, function, initial, duration, intervals):
    state = np.array(initial, dtype=float)
    h = duration / intervals
    for index in range(intervals):
        state = step(function, index * h, state, h)
    return state


class StepFormulaChecks(unittest.TestCase):
    def test_euler_converges_at_first_order_for_exponential(self):
        errors = []
        for intervals in (20, 40):
            result = integrate(integrators.euler_step, lambda t, x: x, [1], 1, intervals)
            errors.append(abs(result[0] - np.e))
        self.assertGreater(errors[0] / errors[1], 1.8)
        self.assertLess(errors[0] / errors[1], 2.1)

    def test_rk4_converges_at_fourth_order_for_exponential(self):
        errors = []
        for intervals in (10, 20):
            result = integrate(integrators.rk4_step, lambda t, x: x, [1], 1, intervals)
            errors.append(abs(result[0] - np.e))
        self.assertGreater(errors[0] / errors[1], 14)
        self.assertLess(errors[0] / errors[1], 17)

    def test_constant_acceleration_updates_coupled_position_and_velocity(self):
        # Exact solution: position = 10 + 3*t + t^2; velocity = 3 + 2*t.
        derivative = lambda t, x: np.array([x[1], 2.0])
        euler = integrate(integrators.euler_step, derivative, [10, 3], 2, 20)
        rk4 = integrate(integrators.rk4_step, derivative, [10, 3], 2, 20)
        np.testing.assert_allclose(rk4, [20, 7], atol=1e-12, rtol=0)
        self.assertAlmostEqual(euler[1], 7)
        self.assertGreater(abs(euler[0] - 20), 0.1)

    def test_time_dependent_rhs_is_evaluated_at_intermediate_times(self):
        # Integral of t^3 from 0 to 2 is 4; RK4 quadrature is exact here.
        derivative = lambda t, x: np.full_like(x, t ** 3)
        result = integrate(integrators.rk4_step, derivative, [0], 2, 4)
        self.assertAlmostEqual(result[0], 4, places=12)

    def test_harmonic_oscillator_accuracy(self):
        derivative = lambda t, x: np.array([x[1], -x[0]])
        result = integrate(integrators.rk4_step, derivative, [1, 0], 2, 40)
        np.testing.assert_allclose(result, [np.cos(2), -np.sin(2)], atol=2e-7, rtol=0)


if __name__ == "__main__":
    unittest.main()
