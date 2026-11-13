# SPDX-License-Identifier: LGPL-2.1-or-later
# Steady adaptations of JSBSim FGPiston.cpp / FGPropeller.cpp at c4558970.
# Copyright (C) 2000 Jon S. Berndt. Engine model: Dave Luff, Ronald Jensen.
# Modified for this project: SI stationary equations, inverse matching and guards.
# Provided without warranty; see LICENSES/JSBSim-LGPL.txt and THIRD_PARTY_NOTICES.md.
"""Stationary motor/propeller matching for the pinned reduced civil C172 model."""
import json
from pathlib import Path
import numpy as np

HP_W = 745.7  # Conversion used by the pinned engine implementation.
INHG_PA = 3386.389
DATA = Path(__file__).resolve().parents[1] / "data" / "c172_propulsion.json"


def bisect(function, low, high, iterations=45):
    a, b = float(function(low)), float(function(high))
    if not np.isfinite(a + b) or a * b > 0:
        raise ValueError("Requested operating point is not bracketed")
    for _ in range(iterations):
        mid = (low + high) / 2
        value = float(function(mid))
        if not np.isfinite(value):
            raise ValueError("Nonfinite operating-point residual")
        if a * value <= 0:
            high, b = mid, value
        else:
            low, a = mid, value
    return (low + high) / 2


class SteadyPropulsion:
    def __init__(self, fuel_air_ratio=0.07786):
        self.data = json.loads(DATA.read_text(encoding="utf-8"))
        self.engine = self.data["engine_xml"]
        self.defaults = self.data["engine_constructor_defaults"]
        self.tables = {name: np.asarray(rows) for name, rows in self.data["propeller_tables"].items()}
        self.mixture = np.asarray(self.data["mixture_efficiency"])
        if not np.isfinite(fuel_air_ratio) or not self.mixture[0, 0] < fuel_air_ratio < self.mixture[-1, 0]:
            raise ValueError("Fuel/air ratio must be inside the source mixture table")
        self.fuel_air_ratio = float(fuel_air_ratio)
        self.mixture_efficiency = float(np.interp(fuel_air_ratio, self.mixture[:, 0], self.mixture[:, 1]))
        # Fixed atmosphere matches the existing 1000 m study. Engine gas constant
        # remains 287.3 J/(kg K), as in the source; airframe ISA uses 287.05.
        self.temperature_K = 288.15 - 0.0065 * 1000
        self.pressure_Pa = 101325 * (self.temperature_K / 288.15) ** (9.80665 / (287.05 * 0.0065))
        self.sound_speed = np.sqrt(1.4 * 287.05 * self.temperature_K)

    def lookup(self, name, value):
        table = self.tables[name]
        if name in ("C_THRUST", "C_POWER") and not table[0, 0] <= value <= table[-1, 0]:
            raise ValueError("Advance ratio outside the source propeller table")
        return float(np.interp(value, table[:, 0], table[:, 1]))

    def propeller(self, rpm, axial_speed, density):
        n, diameter = rpm / 60, self.data["diameter_m"]
        if n <= 0 or axial_speed <= 0 or density <= 0:
            raise ValueError("Positive RPM, axial velocity and density are required")
        advance = axial_speed / (n * diameter)
        tip_mach = np.hypot(np.pi * diameter * n, axial_speed) / self.sound_speed
        thrust = self.lookup("C_THRUST", advance) * self.lookup("CT_MACH", tip_mach) * density * n ** 2 * diameter ** 4
        power = self.lookup("C_POWER", advance) * self.lookup("CP_MACH", tip_mach) * density * n ** 3 * diameter ** 5
        return {"thrust_N": float(thrust), "shaft_power_W": float(power),
                "advance_ratio": float(advance), "tip_mach": float(tip_mach)}

    def engine_point(self, rpm, throttle, axial_speed):
        """Stationary intake, air/fuel flows, mixture and internal engine losses."""
        e, d = self.engine, self.defaults
        p, temperature = self.pressure_Pa, self.temperature_K
        piston_speed = rpm * d["Stroke"] / 360  # ft/s, source convention
        rated_speed = e["maxrpm"] * d["Stroke"] / 360
        ze_rated = d["PeakMeanPistonSpeed_fps"] / rated_speed
        z_airbox = 101320.73 * ze_rated / (e["maxmp"] * INHG_PA) - ze_rated
        z_throttle = (d["PeakMeanPistonSpeed_fps"] / (e["idlerpm"] * d["Stroke"] / 360)
                      * (101320.73 / (e["minmp"] * INHG_PA) - 1) - z_airbox)
        ze = d["PeakMeanPistonSpeed_fps"] / piston_speed
        # Unit ram-air factor; no intake lag in stationary operation.
        total_pressure = p * (1 + 0.2 * (axial_speed / self.sound_speed) ** 2) ** 3.5
        manifold = total_pressure * ze / (ze + z_airbox + (1 - throttle) ** 2 * z_throttle)
        compression = d["CompressionRatio"]
        ratio = min(p / manifold, compression)
        correction = 0.3 / 1.3 + (compression - ratio) / (1.3 * (compression - 1))
        displacement = e["displacement"] * 0.0254 ** 3
        airflow = displacement * (rpm / 60) / 2 * d["volumetric_efficiency"] * correction * manifold / (287.3 * temperature)
        fuel_flow = airflow * self.fuel_air_ratio
        indicated_hp = fuel_flow * 2.2046 * 3600 / e["bsfc"] * self.mixture_efficiency - d["StaticFriction_HP"]
        pmep = (manifold - p) * d["volumetric_efficiency"]
        fmep = -d["FMEPDynamic"] * piston_speed * 0.3048 - d["FMEPStatic"]
        pumping_hp = (pmep + fmep) * displacement * rpm / (e["cycles"] * 22371)
        return {"shaft_power_W": float((indicated_hp + pumping_hp) * HP_W),
                "fuel_flow_kg_s": float(fuel_flow), "manifold_pressure_Pa": float(manifold),
                "indicated_power_W": float(indicated_hp * HP_W),
                "internal_loss_W": float(-pumping_hp * HP_W)}

    def match(self, thrust, axial_speed, density):
        e = self.engine
        rpm = bisect(lambda n: self.propeller(n, axial_speed, density)["thrust_N"] - thrust,
                     e["idlerpm"], e["maxrpm"])
        prop = self.propeller(rpm, axial_speed, density)
        power = prop["shaft_power_W"]
        if power <= 0 or power > e["maxhp"] * HP_W:
            raise ValueError("Required shaft power exceeds the study engine limit")
        throttle = bisect(lambda setting: self.engine_point(rpm, setting, axial_speed)["shaft_power_W"] - power,
                          e["minthrottle"], e["maxthrottle"])
        engine = self.engine_point(rpm, throttle, axial_speed)
        efficiency = thrust * axial_speed / power
        if not 0 < efficiency <= 1 or engine["fuel_flow_kg_s"] <= 0:
            raise ValueError("Nonphysical propulsive efficiency or fuel flow")
        mixture_position = self.fuel_air_ratio * 14.7 / 1.3 * self.pressure_Pa / 101325
        if not 0 < mixture_position <= 1:
            raise ValueError("Chosen mixture cannot be set at this atmospheric condition")
        return {**prop, **engine, "rpm": float(rpm), "throttle": float(throttle),
                "propulsive_efficiency": float(efficiency), "mixture_position": float(mixture_position),
                "thrust_match_error_N": float(prop["thrust_N"] - thrust),
                "power_match_error_W": float(engine["shaft_power_W"] - power)}
