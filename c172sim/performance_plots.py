"""Renderer-independent definitions of cruise-performance figures."""
import numpy as np

COLORS = {"maximum_endurance": "#1781a0", "maximum_range": "#d37e22", "fixed_speed": "#172c3b"}
LABELS = {"maximum_endurance": "Sampled endurance", "maximum_range": "Sampled range", "fixed_speed": "Fixed 50 m/s"}


def plot_specs(result):
    initial = result["fuel_nodes_kg"][0]
    points = [p for p in result["accepted_points"] if p["fuel_remaining_kg"] == initial]
    panels = []
    for key, title, factor in (("alpha_deg", "Balanced angle of attack [deg]", 1),
                               ("thrust_N", "Required axial thrust [N]", 1),
                               ("shaft_power_W", "Required shaft power [kW]", 0.001),
                               ("fuel_kg_h", "Fuel per hour [kg/h] | endurance criterion", 1),
                               ("fuel_kg_km", "Fuel per distance [kg/km] | range criterion", 1),
                               ("propulsive_efficiency", "Propulsive efficiency [-]", 1)):
        series = [{"x": np.array([p["speed_m_s"] for p in points]),
                   "y": np.array([p[key] * factor for p in points]),
                   "label": "Accepted steady flight", "color": "#172c3b", "style": "solid"}]
        for strategy in ("maximum_endurance", "maximum_range"):
            selected = result["profiles"][strategy][0]
            series.append({"x": np.array([selected["speed_m_s"]]),
                           "y": np.array([selected[key] * factor]),
                           "label": LABELS[strategy], "color": COLORS[strategy], "style": "scatter"})
        panels.append({"title": title, "xlabel": "True airspeed [m/s]", "ylabel": "", "series": series})
    curves = {"filename": "cruise_performance.png", "columns": 2,
              "title": "One weight | a family of balanced flight speeds",
              "subtitle": f"Initial fuel {initial:.3f} kg | 1000 m, no wind | each point solves all force/moment balances",
              "panels": panels}
    panels = []
    fixed = result["summaries"]["fixed_speed"]["initial_speed_m_s"]
    for xkey, xscale, xlabel, ykey, yscale, title in (
        ("time_s", 1 / 3600, "Time [h]", "fuel_remaining_kg", 1, "Fuel remaining [kg]"),
        ("time_s", 1 / 3600, "Time [h]", "distance_m", 0.001, "Accumulated cruise distance [km]"),
        ("fuel_remaining_kg", 1, "Fuel remaining [kg]", "speed_m_s", 1, "Selected true airspeed [m/s]"),
        ("fuel_remaining_kg", 1, "Fuel remaining [kg]", "fuel_kg_h", 1, "Fuel flow [kg/h]")):
        series = []
        for strategy, profile in result["profiles"].items():
            label = LABELS[strategy] if strategy != "fixed_speed" else f"Fixed {fixed:g} m/s"
            series.append({"x": np.array([p[xkey] * xscale for p in profile]),
                           "y": np.array([p[ykey] * yscale for p in profile]), "label": label,
                           "color": COLORS[strategy], "style": "solid" if strategy != "fixed_speed" else "dashed"})
        panels.append({"title": title, "xlabel": xlabel, "ylabel": "", "series": series})
    mission = {"filename": "cruise_missions.png", "columns": 2,
               "title": "Equal fuel burned | sampled cruise strategies",
               "subtitle": "Quasi-steady equilibria | fuel/air ratio fixed | instantaneous speed changes | no climb or descent",
               "panels": panels}
    return [curves, mission]
