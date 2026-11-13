"""Compare sampled cruise endurance/range with stationary propulsion and fuel burn."""
import argparse
import csv
import json
from pathlib import Path
import platform
import numpy as np
from c172sim import __version__
from c172sim.mass_properties import FuelLoadCase
from c172sim.propulsion import SteadyPropulsion
from c172sim.performance import study, refinement_check


def save_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ["fuel_remaining_kg", "speed_m_s", "reason"])
        writer.writeheader()
        writer.writerows(rows)


def save_results(directory, result, report):
    directory.mkdir(parents=True, exist_ok=True)
    save_csv(directory / "operating_points.csv", result["accepted_points"])
    save_csv(directory / "rejected_points.csv", result["rejected_points"])
    for strategy, rows in result["profiles"].items():
        save_csv(directory / f"{strategy}.csv", rows)
    (directory / "performance.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    lines = ["# Sampled quasi-steady cruise comparison", "",
             "Constant 1000 m atmosphere, no wind; fuel/air ratio fixed; equal tank depletion.",
             "All strategies burn the same fuel mass. Climb, descent and speed-transition costs are omitted.", "",
             "| Sampled strategy | Duration [h] | Distance [km] | Initial speed [m/s] | Final speed [m/s] | Feasible-edge selection |",
             "|---|---:|---:|---:|---:|---|"]
    for name, row in result["summaries"].items():
        lines.append(f"| {name} | {row['duration_h']:.4f} | {row['distance_km']:.3f} | {row['initial_speed_m_s']:.2f} | {row['final_speed_m_s']:.2f} | {row['edge_selected_at_any_fuel_node']} |")
    lines += ["", "The word maximum refers only to accepted sampled operating points. An edge selection is domain-constrained; a lower-speed optimum outside the model domain cannot be assessed.",
              "Numerical refinement and source-model consistency are not flight-test validation.", ""]
    (directory / "summary.md").write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed-min", type=float, default=26.0)
    parser.add_argument("--speed-max", type=float, default=74.0)
    parser.add_argument("--speed-step", type=float, default=1.0)
    parser.add_argument("--fuel-intervals", type=int, default=12)
    parser.add_argument("--initial-fuel-kg", type=float, help="Default: packaged half-capacity load")
    parser.add_argument("--stop-fuel-kg", type=float, default=10.0, help="Retained fuel, not an operational reserve rule")
    parser.add_argument("--fixed-speed", type=float, default=50.0)
    parser.add_argument("--fuel-air-ratio", type=float, default=0.07786, help="Mass ratio held constant by adjusting mixture")
    parser.add_argument("--skip-refinement", action="store_true")
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "results" / "performance")
    args = parser.parse_args()
    load, propulsion = FuelLoadCase(), SteadyPropulsion(args.fuel_air_ratio)
    initial = load.initial_fuel_kg if args.initial_fuel_kg is None else args.initial_fuel_kg
    values = [initial, args.stop_fuel_kg, args.speed_min, args.speed_max, args.speed_step, args.fixed_speed]
    if (not np.all(np.isfinite(values)) or not 0 <= args.stop_fuel_kg < initial <= load.capacity_kg
            or not 25 < args.speed_min < args.speed_max < 75 or args.speed_step <= 0
            or not args.speed_min <= args.fixed_speed <= args.speed_max or args.fuel_intervals < 2):
        parser.error("Require finite in-domain speeds, positive spacing, initial > stop >= 0 fuel and at least two fuel intervals")

    def run(speed_step, intervals):
        # linspace includes both bounds and never exceeds the requested spacing.
        speeds = np.linspace(args.speed_min, args.speed_max,
                             int(np.ceil((args.speed_max - args.speed_min) / speed_step)) + 1)
        speeds = np.unique(np.append(speeds, args.fixed_speed))
        return study(load, propulsion, speeds, np.linspace(initial, args.stop_fuel_kg, intervals + 1),
                     args.fixed_speed, progress=True)

    result = run(args.speed_step, args.fuel_intervals)
    refinement = {"performed": False}
    if not args.skip_refinement:
        fine = run(args.speed_step / 2, args.fuel_intervals * 2)
        refinement = refinement_check(result, fine)
        result = fine
    report = {"software_version": __version__, "source_commit": propulsion.data["source_commit"],
              "environment": {"python": platform.python_version(), "numpy": np.__version__},
              "propulsion_sources": propulsion.data["sources"],
              "settings": {"initial_fuel_kg": initial, "stop_fuel_kg": args.stop_fuel_kg,
                           "fixed_speed_m_s": args.fixed_speed, "altitude_m": 1000,
                           "fuel_air_ratio": propulsion.fuel_air_ratio,
                           "speed_grid_m_s": result["speed_grid_m_s"], "fuel_nodes_kg": result["fuel_nodes_kg"]},
              "method": "Six force/moment trim balances at every fuel/speed node; bounded NumPy Newton solver; fixed-pitch inverse propeller/engine matching; fuel-coordinate trapezoidal quadrature.",
              "assumptions": ["Fixed 1000 m ISA atmosphere and airframe density; no wind.",
                              "Fuel removed equally from both tanks; loaded CG, tensor and moment arms reassembled.",
                              "Fixed fuel/air ratio, both magnetos, stationary naturally aspirated motor; source internal losses retained.",
                              "Candidate selection on a finite speed grid; instant cost-free equilibrium transitions.",
                              "Analysis stop fuel is a chosen retained amount, not a certified usable-fuel or reserve specification.",
                              "RPM and nominal-power study caps; throttle bounds from XML; no certified operating-envelope claim."],
              "limitations": "Reduced aerodynamic model and finite trim/domain bounds can constrain an optimum. No 6-DOF fuel state, autopilot, climb/descent, acceleration cost, weather or POH validation. Source agreement is not real-aircraft validation.",
              "counts": {"accepted": len(result["accepted_points"]), "rejected": len(result["rejected_points"])},
              "summaries": result["summaries"], "refinement": refinement,
              "maximum_trim_scaled_residual": max(p["trim_scaled_residual"] for p in result["accepted_points"]),
              "maximum_thrust_match_error_N": max(abs(p["thrust_match_error_N"]) for p in result["accepted_points"]),
              "maximum_power_match_error_W": max(abs(p["power_match_error_W"]) for p in result["accepted_points"]),
              "figure_renderer": None if args.no_plots else "Matplotlib"}
    save_results(args.output_dir, result, report)
    if not args.no_plots:
        from c172sim.performance_plots import plot_specs
        from c172sim.analysis_plots import render_matplotlib
        render_matplotlib(args.output_dir, plot_specs(result))
    print(json.dumps(result["summaries"], indent=2))
    print(f"Results: {args.output_dir}")
    if refinement.get("performed") and not refinement["passed"]:
        raise SystemExit("Refinement limit exceeded; inspect the report and refine the grids further")


if __name__ == "__main__":
    main()
