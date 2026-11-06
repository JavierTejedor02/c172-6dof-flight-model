"""Check physical error units and exact candidate/reference timestamp matching."""
from types import SimpleNamespace
import unittest

import numpy as np
from compare_integrators import error_metrics, matched_states


class ComparisonMetricChecks(unittest.TestCase):
    def test_scaled_error_respects_physical_unit_scales(self):
        reference = np.zeros((3, 12))
        states = reference.copy()
        states[1, 0] = 1.0            # One metre per second.
        states[1, 1] = np.deg2rad(1)  # One degree of angle of attack.
        states[1, 4] = np.deg2rad(1)  # One degree per second of pitch rate.
        states[1, 9] = 1.0            # One metre north.
        states[1, 11] = -1.0          # One metre of altitude.
        metrics = error_metrics(states, reference)
        self.assertAlmostEqual(metrics["max_scaled_state_error"], 1)
        self.assertAlmostEqual(metrics["rms_scaled_state_error"], np.sqrt(5 / 36))
        self.assertAlmostEqual(metrics["max_position_error_m"], np.sqrt(2))
        self.assertAlmostEqual(metrics["max_altitude_error_m"], 1)
        self.assertAlmostEqual(metrics["max_pitch_rate_error_deg_s"], 1)
        self.assertAlmostEqual(metrics["max_abs_error_by_state"]["q_rad_s"], np.deg2rad(1))

    def test_reference_sampling_requires_exact_timestamps(self):
        reference = SimpleNamespace(time=np.array([0.0, 0.1, 0.2]),
                                    states=np.arange(36).reshape(3, 12))
        np.testing.assert_array_equal(matched_states(reference, [0, 0.2]),
                                      reference.states[[0, 2]])
        for times in ([0.15], [0.3]):
            with self.subTest(times=times), self.assertRaises(RuntimeError):
                matched_states(reference, times)


if __name__ == "__main__":
    unittest.main()
