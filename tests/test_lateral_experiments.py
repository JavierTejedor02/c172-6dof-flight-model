"""Independent lateral pulses, input conventions, trim reuse and refinement."""
from dataclasses import asdict, replace
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from c172sim import Aircraft
from c172sim.dynamics import rhs
from c172sim.experiments import checked_trim_state, lateral_responses, pulse_segments, step_refinement_check

TRIM_FILE = Path(__file__).resolve().parents[1] / "results" / "longitudinal" / "trim_and_run.json"


class LateralExperimentChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.aircraft = Aircraft()
        cls.state, cls.controls = checked_trim_state(cls.aircraft, TRIM_FILE)

    def test_pulse_changes_only_the_selected_surface_and_restores_trim(self):
        for surface in ("aileron", "rudder"):
            segments = pulse_segments(self.controls, surface, 1.0)
            self.assertEqual([duration for duration, _ in segments], [2, 1, 27])
            self.assertEqual(segments[0][1], self.controls)
            self.assertEqual(segments[-1][1], self.controls)
            for name, value in asdict(self.controls).items():
                actual = getattr(segments[1][1], name)
                expected = value + np.deg2rad(1) if name == surface + "_rad" else value
                self.assertAlmostEqual(actual, expected)
        with self.assertRaises(ValueError):
            pulse_segments(self.controls, "rudder", 1, duration=3)

    def test_initial_acceleration_respects_model_control_signs(self):
        baseline = rhs(0, self.state, self.aircraft, self.controls)
        aileron = pulse_segments(self.controls, "aileron", 1)[1][1]
        rudder = pulse_segments(self.controls, "rudder", 1)[1][1]
        da = rhs(0, self.state, self.aircraft, aileron) - baseline
        dr = rhs(0, self.state, self.aircraft, rudder) - baseline
        self.assertGreater(da[3], 0)  # Positive effective aileron gives positive roll acceleration.
        self.assertLess(dr[5], 0)     # Positive model rudder gives negative yaw acceleration.
        self.assertGreater(abs(da[5]), 1e-5)  # Aileron also changes yaw dynamics.
        self.assertGreater(abs(dr[3]), 1e-5)  # Rudder also changes roll dynamics.

    def test_independent_trajectories_and_step_refinement(self):
        coarse = lateral_responses(self.aircraft, self.state, self.controls,
                                   duration=8, method="RK4", step_size=0.02)
        fine = lateral_responses(self.aircraft, self.state, self.controls,
                                 duration=8, method="RK4", step_size=0.01)
        self.assertTrue(step_refinement_check(coarse, fine, 0.02)["passed"])
        invalid_fine = dict(fine)
        invalid_states = fine["aileron"].states.copy()
        invalid_states[:, 9] += 1
        invalid_fine["aileron"] = replace(fine["aileron"], states=invalid_states)
        with self.assertRaisesRegex(RuntimeError, "refinement failed"):
            step_refinement_check(coarse, invalid_fine, 0.02)
        for name in ("aileron", "rudder"):
            with self.subTest(surface=name):
                np.testing.assert_array_equal(coarse[name].states[0], self.state)
                np.testing.assert_allclose(coarse[name].time, fine[name].time[::2], atol=1e-13)
                difference = np.abs(coarse[name].states - fine[name].states[::2])
                self.assertLess(np.max(difference[:, :9]), 1e-5)
                self.assertLess(np.max(difference[:, 9:]), 1e-3)
                self.assertTrue(np.all(np.diff(coarse[name].time) > 0))
                self.assertGreater(np.max(np.abs(coarse[name].states[:, 2])), 1e-4)
        zero = lateral_responses(self.aircraft, self.state, self.controls, duration=4,
                                 aileron_deg=0, rudder_deg=0, method="RK4", step_size=0.02)
        for name in ("aileron", "rudder"):
            np.testing.assert_allclose(zero[name].states, zero["equilibrium"].states,
                                       atol=1e-7, rtol=0)

    def test_recorded_trim_rejects_mismatched_properties_or_balance(self):
        saved = json.loads(TRIM_FILE.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as temporary:
            filename = Path(temporary) / "trim.json"
            for field in ("mass_kg", "density_kg_m3"):
                changed = dict(saved)
                changed[field] += 1
                filename.write_text(json.dumps(changed), encoding="utf-8")
                with self.subTest(field=field), self.assertRaisesRegex(ValueError, "does not match"):
                    checked_trim_state(self.aircraft, filename)
            changed = json.loads(json.dumps(saved))
            changed["trim_controls"]["thrust_N"] += 100
            filename.write_text(json.dumps(changed), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "no longer balances"):
                checked_trim_state(self.aircraft, filename)


if __name__ == "__main__":
    unittest.main()
