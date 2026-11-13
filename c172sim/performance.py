"""Fuel-coordinate, quasi-steady civil missions and sampled speed selection."""
import numpy as np
from .frames import body_velocity
from .mass_properties import FuelLoadCase
from .propulsion import SteadyPropulsion
from .trim import level_trim

STRATEGIES = ("maximum_endurance", "maximum_range", "fixed_speed")


def operating_point(aircraft, propulsion, speed):
    trim = level_trim(aircraft, speed, altitude=1000.0, solver="newton")
    axial_speed = body_velocity(*trim.state[:3])[0]
    match = propulsion.match(trim.controls.thrust_N, axial_speed, aircraft.rho)
    cl, cd = aircraft.coefficients(trim.state, trim.controls)[:2]
    q_area = 0.5 * aircraft.rho * speed ** 2 * aircraft.area
    return {"speed_m_s": float(speed), "mass_kg": aircraft.mass,
            "alpha_deg": float(np.rad2deg(trim.state[1])),
            "bank_deg": float(np.rad2deg(trim.state[6])),
            "elevator_deg": float(np.rad2deg(trim.controls.elevator_rad)),
            "aileron_deg": float(np.rad2deg(trim.controls.aileron_rad)),
            "rudder_deg": float(np.rad2deg(trim.controls.rudder_rad)),
            "thrust_N": trim.controls.thrust_N, "drag_N": float(q_area * cd),
            "lift_N": float(q_area * cl), "weight_N": aircraft.mass * aircraft.gravity,
            "cg_structural_x_m": aircraft.data["cg_structural_m"][0],
            "cg_structural_y_m": aircraft.data["cg_structural_m"][1],
            "cg_structural_z_m": aircraft.data["cg_structural_m"][2],
            "trim_scaled_residual": trim.scaled_residual_norm,
            "fuel_kg_h": match["fuel_flow_kg_s"] * 3600,
            "fuel_kg_km": match["fuel_flow_kg_s"] * 1000 / speed, **match}


def integrate_profile(points):
    """Trapezoidal quadrature of dt=-dm_f/flow and ds=-V dm_f/flow.

    Fuel is the independent coordinate; no 6-DOF dynamic integration occurs.
    Every node is a newly balanced configuration, including the final node.
    """
    if len(points) < 2:
        raise ValueError("At least two mission nodes are required")
    result = []
    time, distance = 0.0, 0.0
    for index, point in enumerate(points):
        if point["fuel_flow_kg_s"] <= 0 or not np.isfinite(point["fuel_flow_kg_s"]):
            raise ValueError("Positive finite fuel flow is required")
        if index:
            previous = points[index - 1]
            fuel_used = previous["fuel_remaining_kg"] - point["fuel_remaining_kg"]
            if fuel_used <= 0:
                raise ValueError("Mission fuel nodes must strictly decrease")
            time += fuel_used * 0.5 * (1 / previous["fuel_flow_kg_s"] + 1 / point["fuel_flow_kg_s"])
            distance += fuel_used * 0.5 * (previous["speed_m_s"] / previous["fuel_flow_kg_s"]
                                          + point["speed_m_s"] / point["fuel_flow_kg_s"])
        result.append({**point, "time_s": float(time), "distance_m": float(distance)})
    return result


def study(load_case, propulsion, speeds, fuel_nodes, fixed_speed=50.0, progress=False):
    speeds, fuel_nodes = np.asarray(speeds, dtype=float), np.asarray(fuel_nodes, dtype=float)
    if (speeds.ndim != 1 or not len(speeds) or not np.all(np.isfinite(speeds))
            or np.any(np.diff(speeds) <= 0) or min(speeds) <= 25 or max(speeds) >= 75):
        raise ValueError("Ordered distinct speed samples must lie strictly between 25 and 75 m/s")
    if (fuel_nodes.ndim != 1 or len(fuel_nodes) < 2 or not np.all(np.isfinite(fuel_nodes))
            or np.any(np.diff(fuel_nodes) >= 0)):
        raise ValueError("At least two strictly decreasing finite fuel nodes are required")
    if fixed_speed not in speeds:
        raise ValueError("Fixed comparison speed must be included in the speed grid")
    accepted, rejected = [], []
    selected = {strategy: [] for strategy in STRATEGIES}
    for fuel in fuel_nodes:
        aircraft = load_case.aircraft(fuel)
        points = []
        for speed in speeds:
            try:
                point = {"fuel_remaining_kg": float(fuel), **operating_point(aircraft, propulsion, speed)}
                points.append(point)
                accepted.append(point)
            except (RuntimeError, ValueError) as error:
                rejected.append({"fuel_remaining_kg": float(fuel), "speed_m_s": float(speed), "reason": str(error)})
        if not points:
            raise RuntimeError(f"No feasible speed at fuel={fuel:g} kg")
        choices = {"maximum_endurance": min(points, key=lambda p: p["fuel_flow_kg_s"]),
                   "maximum_range": min(points, key=lambda p: p["fuel_kg_km"]),
                   "fixed_speed": next((p for p in points if p["speed_m_s"] == fixed_speed), None)}
        if choices["fixed_speed"] is None:
            raise RuntimeError("Fixed-speed comparator is infeasible at a mission fuel node")
        for strategy, point in choices.items():
            selected[strategy].append({**point,
                "at_sampled_feasible_speed_edge": point["speed_m_s"] in (points[0]["speed_m_s"], points[-1]["speed_m_s"])})
        if progress:
            print(f"Fuel {fuel:.3f} kg: {len(points)}/{len(speeds)} feasible speeds", flush=True)
    profiles = {strategy: integrate_profile(points) for strategy, points in selected.items()}
    summaries = {strategy: {"duration_h": points[-1]["time_s"] / 3600,
                           "distance_km": points[-1]["distance_m"] / 1000,
                           "fuel_used_kg": float(fuel_nodes[0] - fuel_nodes[-1]),
                           "initial_speed_m_s": points[0]["speed_m_s"],
                           "final_speed_m_s": points[-1]["speed_m_s"],
                           "edge_selected_at_any_fuel_node": any(p["at_sampled_feasible_speed_edge"] for p in points)}
                 for strategy, points in profiles.items()}
    return {"speed_grid_m_s": speeds.tolist(), "fuel_nodes_kg": fuel_nodes.tolist(),
            "accepted_points": accepted, "rejected_points": rejected,
            "profiles": profiles, "summaries": summaries}


def refinement_check(coarse, fine, limit=0.01):
    differences = {}
    for strategy in STRATEGIES:
        differences[strategy] = {key: abs(coarse["summaries"][strategy][key] / fine["summaries"][strategy][key] - 1)
                                 for key in ("duration_h", "distance_km")}
    maximum = max(v for row in differences.values() for v in row.values())
    return {"performed": True, "passed": bool(maximum < limit), "relative_limit": limit,
            "relative_changes": differences, "max_relative_change": maximum,
            "scope": "Fuel-node spacing and speed spacing halved together; sampled optima and fuel quadrature, not a global-optimality proof."}
