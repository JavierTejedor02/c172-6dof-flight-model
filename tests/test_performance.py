"""Source identity, steady balances, propulsion closure and fuel quadrature."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
import numpy as np
from c172sim import Aircraft, level_trim
from c172sim.dynamics import rhs
from c172sim.mass_properties import FuelLoadCase, DATA_DIR
from c172sim.propulsion import SteadyPropulsion, HP_W
from c172sim.performance import operating_point, integrate_profile, study, refinement_check
from run_performance_analysis import save_results


class PerformanceChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.load = FuelLoadCase()
        cls.propulsion = SteadyPropulsion()

    def test_reference_load_reconstructed_and_snapshots_independent(self):
        original = Aircraft()
        reproduced = self.load.aircraft(self.load.initial_fuel_kg)
        self.assertAlmostEqual(original.mass, reproduced.mass, places=10)
        for name in ("inertia", "aero_offset", "thrust_offset"):
            np.testing.assert_allclose(getattr(original, name), getattr(reproduced, name), atol=1e-12, rtol=0)
        light = self.load.aircraft(10)
        self.assertAlmostEqual(original.mass - light.mass, self.load.initial_fuel_kg - 10)
        self.assertGreater(np.min(np.linalg.eigvalsh(light.inertia)), 0)
        self.assertGreater(np.linalg.norm(original.inertia - light.inertia), 1)
        self.assertGreater(np.linalg.norm(original.aero_offset - light.aero_offset), 0)
        np.testing.assert_array_equal(original.inertia, self.load.template.inertia)
        for fuel in (-1, self.load.capacity_kg + 1, np.nan):
            with self.assertRaises(ValueError): self.load.aircraft(fuel)

    def test_numpy_trim_matches_recorded_reference_and_balances_other_loads(self):
        record = json.loads((DATA_DIR.parent / "results/longitudinal/trim_and_run.json").read_text(encoding="utf-8"))
        for fuel, speed in ((self.load.initial_fuel_kg, 50), (10, 34), (10, 55), (self.load.initial_fuel_kg, 40)):
            aircraft = self.load.aircraft(fuel)
            trim = level_trim(aircraft, speed, solver="newton")
            self.assertLess(np.max(np.abs(rhs(0, trim.state, aircraft, trim.controls)[:9])), 1e-7)
            self.assertLess(abs(rhs(0, trim.state, aircraft, trim.controls)[11]), 1e-9)
            self.assertLess(np.max(np.abs(trim.force_residual_N)), 1e-5)
            self.assertLess(np.max(np.abs(trim.moment_residual_Nm)), 1e-5)
            if fuel == self.load.initial_fuel_kg and speed == 50:
                self.assertAlmostEqual(trim.controls.thrust_N, record["trim_controls"]["thrust_N"], places=5)
                self.assertAlmostEqual(trim.state[1], record["trim_state"]["alpha_rad"], places=8)

    def test_same_weight_has_several_balanced_speeds_and_different_costs(self):
        aircraft = self.load.aircraft(self.load.initial_fuel_kg)
        points = [operating_point(aircraft, self.propulsion, speed) for speed in (38, 40, 50)]
        self.assertGreater(points[0]["alpha_deg"], points[1]["alpha_deg"])
        self.assertGreater(points[1]["alpha_deg"], points[2]["alpha_deg"])
        self.assertEqual(min(points, key=lambda p: p["fuel_kg_h"])["speed_m_s"], 38)
        self.assertEqual(min(points, key=lambda p: p["fuel_kg_km"])["speed_m_s"], 40)

    def test_propeller_coefficients_and_engine_internal_losses(self):
        prop = self.propulsion
        rpm, speed, density = 2400, 40, 1.1
        point = prop.propeller(rpm, speed, density)
        n, diameter = rpm / 60, prop.data["diameter_m"]
        self.assertAlmostEqual(point["thrust_N"] / (density * n ** 2 * diameter ** 4),
                               prop.lookup("C_THRUST", speed / (n * diameter)), places=12)
        engine = prop.engine_point(rpm, 0.75, speed)
        self.assertGreater(engine["internal_loss_W"], 0)
        self.assertAlmostEqual(engine["shaft_power_W"], engine["indicated_power_W"] - engine["internal_loss_W"], places=8)
        # The XML coefficient is not an effective shaft BSFC.
        naive_fuel_flow = engine["shaft_power_W"] / HP_W * prop.engine["bsfc"] / (2.2046 * 3600)
        self.assertGreater(engine["fuel_flow_kg_s"], naive_fuel_flow)

    def test_inverse_matches_close_and_infeasible_points_rejected(self):
        for speed, thrust in ((34, 886), (40, 830), (50, 1013), (60, 1353)):
            point = self.propulsion.match(thrust, speed, 1.1116425003060326)
            self.assertLess(abs(point["thrust_match_error_N"]), 1e-7)
            self.assertLess(abs(point["power_match_error_W"]), 1e-6)
            self.assertTrue(0 < point["propulsive_efficiency"] < 1)
            self.assertTrue(550 <= point["rpm"] <= 2700)
        with self.assertRaises(ValueError): self.propulsion.match(5000, 60, 1.1116425003060326)
        with self.assertRaises(RuntimeError): level_trim(self.load.aircraft(59), 28, solver="newton")

    def test_fuel_coordinate_quadrature_matches_analytic_constant_flow(self):
        points = [{"fuel_remaining_kg": f, "fuel_flow_kg_s": 0.01, "speed_m_s": 40} for f in (20, 15, 10)]
        final = integrate_profile(points)[-1]
        self.assertEqual(final["time_s"], 1000)
        self.assertEqual(final["distance_m"], 40000)
        with self.assertRaises(ValueError): integrate_profile(list(reversed(points)))

    def test_short_study_objectives_mass_update_refinement_and_exports(self):
        result = study(self.load, self.propulsion, [34, 40, 50], [20, 15, 10])
        profiles = result["profiles"]
        self.assertGreaterEqual(profiles["maximum_endurance"][-1]["time_s"], profiles["maximum_range"][-1]["time_s"])
        self.assertGreaterEqual(profiles["maximum_range"][-1]["distance_m"], profiles["maximum_endurance"][-1]["distance_m"])
        for profile in profiles.values():
            self.assertEqual(profile[0]["mass_kg"] - profile[-1]["mass_kg"], 10)
            self.assertEqual(profile[-1]["fuel_remaining_kg"], 10)
        self.assertTrue(refinement_check(result, result)["passed"])
        with TemporaryDirectory() as directory:
            save_results(Path(directory), result, {"summaries": result["summaries"]})
            data = json.loads((Path(directory) / "performance.json").read_text(encoding="utf-8"))
            self.assertEqual(data["summaries"], result["summaries"])
            self.assertTrue((Path(directory) / "rejected_points.csv").exists())


if __name__ == "__main__":
    unittest.main()
