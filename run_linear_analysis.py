"""Linearize checked trim, inspect candidate modes and compare pulse responses."""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path
import platform

import numpy as np
from c172sim import Aircraft, __version__, level_trim, simulate_segments
from c172sim.dynamics import STATE_NAMES
from c172sim.experiments import checked_trim_state, pulse_segments
from c172sim.linearization import (
    INPUT_NAMES, STATE_SCALES, linearize_aircraft,
    matrix_refinement_check, mode_analysis, simulate_perturbations,
)


def comparison_metrics(predicted, nonlinear):
    difference = predicted - nonlinear
    dynamic_error = np.abs(difference[:, :8]) / STATE_SCALES[:8]
    response = float(np.max(np.abs(nonlinear[:, :8]) / STATE_SCALES[:8]))
    return {
        "max_dynamic_scaled_error": float(np.max(dynamic_error)),
        "rms_dynamic_scaled_error": float(np.sqrt(np.mean(dynamic_error ** 2))),
        "relative_to_peak_dynamic_response": float(np.max(dynamic_error) / max(response, 1e-12)),
        "peak_dynamic_response_scaled": response,
        "max_altitude_error_m": float(np.max(np.abs(difference[:, 11]))),
        "max_abs_error_by_state": dict(zip(STATE_NAMES, np.max(np.abs(difference), axis=0).tolist())),
    }


def compare_case(aircraft, model, baseline, surface, increment_deg, duration, step_size):
    segments = pulse_segments(model.controls, surface, increment_deg, duration)
    nonlinear = simulate_segments(aircraft, model.state, segments, method="RK4", step_size=step_size)
    central = simulate_perturbations(model, segments, step_size, directional=False)
    directional = simulate_perturbations(model, segments, step_size, directional=True)
    if not (np.array_equal(nonlinear.time, central.time)
            and np.array_equal(nonlinear.time, directional.time)
            and np.array_equal(nonlinear.time, baseline.time)):
        raise RuntimeError("Comparison time grids must match exactly")
    nonlinear_delta = nonlinear.states - baseline.states
    report = {"surface": surface, "increment_deg": increment_deg,
              "central_lti": comparison_metrics(central.states, nonlinear_delta),
              "directional": comparison_metrics(directional.states, nonlinear_delta),
              "nfev_nonlinear": nonlinear.nfev,
              "nfev_each_approximation": central.nfev}
    curves = {"time": nonlinear.time, "nonlinear_delta": nonlinear_delta,
              "central_lti_delta": central.states, "directional_delta": directional.states}
    return report, curves


def run_analysis(aircraft, state, controls, amplitudes=(0.25, 0.5, 1.0),
                 duration=30.0, step_size=0.01, check_step=True):
    if (not np.isfinite(duration) or duration <= 3 or not np.isfinite(step_size) or step_size <= 0
            or not len(amplitudes) or not np.all(np.isfinite(amplitudes)) or min(amplitudes) <= 0):
        raise ValueError("Duration > 3 s, positive step and positive finite amplitudes are required")
    amplitudes = sorted(set(float(value) for value in amplitudes))
    model = linearize_aircraft(aircraft, state, controls, step_scale=0.5)
    jacobian_check = matrix_refinement_check(linearize_aircraft(aircraft, state, controls), model)
    # Identical segment boundaries are used even for the zero-input baseline.
    baseline_segments = pulse_segments(controls, "elevator", 0.0, duration)
    baseline = simulate_segments(aircraft, state, baseline_segments, method="RK4", step_size=step_size)
    if np.max(np.abs(baseline.states[:, :9] - state[:9])) > 1e-6:
        raise RuntimeError("The unperturbed state does not persist at trim")
    cases, curves = [], {}
    for surface in ("elevator", "aileron", "rudder"):
        for amplitude in amplitudes:
            for sign in (1, -1):
                increment = sign * amplitude
                key = f"{surface}_{'positive' if sign > 0 else 'negative'}_{amplitude:.12g}"
                if key in curves:
                    raise ValueError("Amplitudes must be distinct at twelve significant digits")
                case, values = compare_case(aircraft, model, baseline, surface, increment, duration, step_size)
                case["case"] = key
                cases.append(case)
                curves[key] = values
                print(f"{surface:8s} {increment:+g} deg: compared", flush=True)

    refinement = {"performed": bool(check_step)}
    if check_step:
        half_baseline = simulate_segments(aircraft, state, baseline_segments, method="RK4", step_size=step_size / 2)
        differences = {}
        for surface in ("elevator", "aileron", "rudder"):
            key = next(row["case"] for row in cases
                       if row["surface"] == surface and row["increment_deg"] == max(amplitudes))
            _, fine = compare_case(aircraft, model, half_baseline, surface,
                                   max(amplitudes), duration, step_size / 2)
            coarse = curves[key]
            # Shared grid nodes can differ by roundoff when a final step shortens.
            indices = np.minimum(np.searchsorted(fine["time"], coarse["time"]), len(fine["time"]) - 1)
            previous = np.maximum(indices - 1, 0)
            indices = np.where(np.abs(fine["time"][previous] - coarse["time"])
                               < np.abs(fine["time"][indices] - coarse["time"]), previous, indices)
            if not np.allclose(fine["time"][indices], coarse["time"], rtol=0, atol=1e-12):
                raise RuntimeError("Step-refinement grids do not share the requested nodes")
            per_method = {}
            for name in ("nonlinear_delta", "central_lti_delta", "directional_delta"):
                difference = np.max(np.abs(coarse[name] - fine[name][indices]), axis=0)
                if (not np.all(np.isfinite(difference)) or np.max(difference[:9]) >= 1e-5
                        or np.max(difference[9:]) >= 1e-3):
                    raise RuntimeError(f"Integration refinement failed: {surface}/{name}; reduce step")
                per_method[name] = difference.tolist()
            differences[surface] = per_method
        refinement.update(passed=True, coarse_step_s=step_size, fine_step_s=step_size / 2,
                          checked_cases="Positive largest amplitude for each surface",
                          max_abs_differences_by_state=differences,
                          first_nine_state_limit=1e-5, position_limit_m=1e-3)

    report = {
        "software_version": __version__, "source_commit": aircraft.data["source_commit"],
        "environment": {"python": platform.python_version(), "numpy": np.__version__},
        "trim_state": dict(zip(STATE_NAMES, model.state.tolist())), "trim_controls": asdict(controls),
        "experiment": {"duration_s": duration, "amplitudes_deg": amplitudes,
                       "pulse_interval_s": [2, 3], "both_signs": True,
                       "method": "RK4", "step_size_s": step_size,
                       "reference": "Nonlinear trajectory minus the common nonlinear trim-control baseline"},
        "state_names": list(STATE_NAMES), "input_names": list(INPUT_NAMES),
        "error_scales_native_units": dict(zip(STATE_NAMES, STATE_SCALES.tolist())),
        "error_definition": "Sampled perturbation errors. Dynamic metrics use the first eight states scaled by 1 m/s, 1 deg or 1 deg/s. Position errors are recorded separately.",
        "jacobian_refinement": jacobian_check, "integration_refinement": refinement,
        "nonsmooth_policy": model.diagnostics["interpretation"],
        "cases": cases,
        "limitations": "Internal consistency of this reduced model, not independent aircraft validation. Central-LTI spectra are approximation spectra at the beta cusp; directional models are piecewise linear, not LTI. Longitudinal/lateral labels refer to inputs and modal participation; the coupled matrices are retained.",
    }
    return model, report, curves


def write_matrix(filename, matrix, column_names):
    with Path(filename).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["state_derivative"] + list(column_names))
        writer.writerows([name + "/s", *row] for name, row in zip(STATE_NAMES, matrix))


def save_analysis(output_dir, model, report, curves):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    write_matrix(output_dir / "A.csv", model.A, STATE_NAMES)
    write_matrix(output_dir / "B.csv", model.B, INPUT_NAMES)
    for sign, name in ((1, "positive_beta"), (-1, "negative_beta")):
        sector = model.A.copy()
        sector[:, 2] += sign * model.cusp
        write_matrix(output_dir / f"A_{name}.csv", sector, STATE_NAMES)
    linearization = {
        "software_version": __version__, "source_commit": report["source_commit"],
        "state_names": list(STATE_NAMES), "input_names": list(INPUT_NAMES),
        "A": model.A.tolist(), "B": model.B.tolist(), "cusp": model.cusp.tolist(),
        "trim_state": report["trim_state"], "trim_controls": report["trim_controls"],
        "diagnostics": model.diagnostics, "finite_difference_check": report["jacobian_refinement"],
        "equations": {"central_lti": "delta_x_dot = A delta_x + B delta_u",
                      "directional": "delta_x_dot = A delta_x + B delta_u + cusp abs(delta_beta)"},
    }
    modes = {sector: mode_analysis(model, sector)
             for sector in ("central", "positive_beta", "negative_beta")}
    for filename, payload in (("linearization.json", linearization), ("modes.json", modes),
                              ("comparison.json", report)):
        (output_dir / filename).write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    table = ["# Candidate dynamic modes", "", "Eight dynamic states are retained. Heading and position kinematics are excluded.",
             "Labels are candidates; frozen-sector eigenvalues do not prove stability of the nonsmooth aircraft model.", ""]
    for sector, analysis in modes.items():
        table += [f"## {sector}", "", "| Candidate | Re(lambda) [1/s] | Im(lambda) [rad/s] | Damping ratio | Period [s] |", "|---|---:|---:|---:|---:|"]
        for row in analysis["modes"]:
            period = f"{row['period_s']:.6g}" if row["period_s"] is not None else "-"
            damping = f"{row['damping_ratio']:.6g}" if row["damping_ratio"] is not None else "-"
            table.append(f"| {row['candidate_label']} | {row['eigenvalue_real_s_inv']:.6g} | {row['eigenvalue_imag_s_inv']:.6g} | {damping} | {period} |")
        table.append("")
    (output_dir / "modes.md").write_text("\n".join(table), encoding="utf-8")
    with (output_dir / "amplitude_sweep.csv").open("w", newline="", encoding="utf-8") as stream:
        fields = ["surface", "increment_deg", "approximation", "max_dynamic_scaled_error",
                  "rms_dynamic_scaled_error", "relative_to_peak_dynamic_response", "max_altitude_error_m"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for case in report["cases"]:
            for approximation in ("central_lti", "directional"):
                writer.writerow({"surface": case["surface"], "increment_deg": case["increment_deg"],
                                 "approximation": approximation,
                                 **{name: case[approximation][name] for name in fields[3:]}})
    archive = {"state_names": np.array(STATE_NAMES)}
    for key, case in curves.items():
        archive.update({key + "__" + name: values for name, values in case.items()})
    np.savez_compressed(output_dir / "perturbation_trajectories.npz", **archive)
    # Human-readable representative traces for each control; all cases are in NPZ.
    for surface in ("elevator", "aileron", "rudder"):
        key = max((case for case in report["cases"] if case["surface"] == surface),
                  key=lambda case: case["increment_deg"])["case"]
        values = curves[key]
        matrix = np.column_stack([values["time"], values["nonlinear_delta"],
                                  values["central_lti_delta"], values["directional_delta"]])
        header = ["time_s"] + [prefix + name for prefix in ("nonlinear_", "central_lti_", "directional_") for name in STATE_NAMES]
        np.savetxt(output_dir / f"{surface}_comparison.csv", matrix, delimiter=",", comments="",
                   header=",".join(header), fmt="%.12g")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trim-file", type=Path, help="Use a checked recorded trim instead of recomputing it")
    parser.add_argument("--amplitudes", type=float, nargs="+", default=[0.25, 0.5, 1.0])
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--step-size", type=float, default=0.01)
    parser.add_argument("--skip-step-check", action="store_true", help="Skip the positive largest-amplitude integration refinement")
    parser.add_argument("--no-plots", action="store_true", help="Write numerical results without importing Matplotlib")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "results" / "linear_analysis")
    args = parser.parse_args()
    if (not np.isfinite(args.duration) or args.duration <= 3 or not np.isfinite(args.step_size)
            or args.step_size <= 0 or not np.all(np.isfinite(args.amplitudes)) or min(args.amplitudes) <= 0):
        parser.error("Duration must exceed 3 s; step and amplitudes must be finite and positive")
    aircraft = Aircraft()
    if args.trim_file is None:
        trim = level_trim(aircraft)
        state, controls = trim.state, trim.controls
    else:
        state, controls = checked_trim_state(aircraft, args.trim_file)
    model, report, curves = run_analysis(aircraft, state, controls, args.amplitudes,
                                         args.duration, args.step_size, not args.skip_step_check)
    report["trim_source"] = "Recomputed trim" if args.trim_file is None else f"Checked record: {args.trim_file.name}"
    report["figure_renderer"] = None if args.no_plots else "Matplotlib"
    save_analysis(args.output_dir, model, report, curves)
    if not args.no_plots:
        from c172sim.analysis_plots import plot_specs, render_matplotlib
        render_matplotlib(args.output_dir, plot_specs(report, curves, model))
    print(f"Results: {args.output_dir}")
    print(f"Finite-difference refinement: {report['jacobian_refinement']['passed']}")
    for mode in mode_analysis(model)["modes"]:
        print(f"{mode['candidate_label']}: {mode['eigenvalue_real_s_inv']:+.6g} +/- {mode['eigenvalue_imag_s_inv']:.6g}j [1/s]")


if __name__ == "__main__":
    main()
