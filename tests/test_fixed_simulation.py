"""Fixed-step flight checks, shared control boundaries, and comparison sampling."""
from dataclasses import replace
from importlib.util import find_spec
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np
from c172sim import Aircraft, Controls, simulate_segments


class FixedSimulationChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.aircraft = Aircraft()
        # Use the existing recorded equilibrium as a reproducible initial state.
        # These integrator tests do not need to run an optimizer first.
        saved = json.loads((Path(__file__).resolve().parents[1] /
                            "results" / "trim_and_run.json").read_text(encoding="utf-8"))
        cls.trim = SimpleNamespace(state=np.array(list(saved["trim_state"].values())),
                                   controls=Controls(**saved["trim_controls"]))

    def test_fractional_steps_land_on_control_boundaries(self):
        pulse = replace(self.trim.controls, elevator_rad=self.trim.controls.elevator_rad - 0.001)
        segments = [(0.13, self.trim.controls), (0.17, pulse)]
        for method, evaluations in (("EULER", 1), ("RK4", 4)):
            with self.subTest(method=method):
                sim = simulate_segments(self.aircraft, self.trim.state, segments,
                                        method=method, step_size=0.05)
                self.assertTrue(np.all(np.diff(sim.time) > 0))
                self.assertAlmostEqual(sim.time[-1], 0.30)
                self.assertEqual(np.count_nonzero(sim.time == 0.13), 1)
                self.assertEqual(sim.nfev, evaluations * (len(sim.time) - 1))
                boundary = np.flatnonzero(sim.time == 0.13)[0]
                self.assertEqual(sim.controls[boundary, 0], self.trim.controls.elevator_rad)
                self.assertEqual(sim.controls[boundary + 1, 0], pulse.elevator_rad)

    def test_fixed_step_is_independent_of_adaptive_output_interval(self):
        for method in ("EULER", "RK4"):
            sim = simulate_segments(self.aircraft, self.trim.state, [(0.2, self.trim.controls)],
                                    method=method, step_size=0.1, dt=0.001)
            np.testing.assert_allclose(sim.time, [0, 0.1, 0.2], atol=1e-15)

    @unittest.skipUnless(find_spec("scipy") is not None, "SciPy is required for the DOP853 reference")
    def test_refined_steps_reduce_flight_trajectory_error(self):
        pulse = replace(self.trim.controls, elevator_rad=self.trim.controls.elevator_rad - np.deg2rad(0.5))
        segments = [(2.0, self.trim.controls), (1.0, pulse), (1.0, self.trim.controls)]
        for method in ("EULER", "RK4"):
            errors = []
            for step in (0.02, 0.01):
                sim = simulate_segments(self.aircraft, self.trim.state, segments,
                                        method=method, step_size=step)
                ref = simulate_segments(self.aircraft, self.trim.state, segments,
                                        output_times=sim.time, rtol=1e-12, atol=1e-14,
                                        max_step=0.01)
                np.testing.assert_array_equal(sim.time, ref.time)
                errors.append(np.max(np.abs(sim.states[:, 7] - ref.states[:, 7])))
            with self.subTest(method=method):
                self.assertLess(errors[1], 0.8 * errors[0])

    def test_fixed_methods_stop_on_domain_exit(self):
        state = self.trim.state.copy()
        state[0] = 74.99
        controls = replace(self.trim.controls, thrust_N=100000)
        for method in ("EULER", "RK4"):
            with self.subTest(method=method), self.assertRaisesRegex(RuntimeError, "domain"):
                simulate_segments(self.aircraft, state, [(1, controls)],
                                  method=method, step_size=0.05)

    def test_invalid_method_steps_durations_and_sampling_are_rejected(self):
        segments = [(1, self.trim.controls)]
        for options in ({"method": "UNKNOWN"}, {"step_size": np.nan},
                        {"step_size": 0}, {"dt": np.inf},
                        {"output_times": [0.2, 0.1]},
                        {"method": "RK4", "output_times": [0, 1]}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                simulate_segments(self.aircraft, self.trim.state, segments, **options)
        with self.assertRaises(ValueError):
            simulate_segments(self.aircraft, self.trim.state, [(np.nan, self.trim.controls)])


if __name__ == "__main__":
    unittest.main()
