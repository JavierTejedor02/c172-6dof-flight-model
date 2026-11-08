"""Compare Euler, RK4 and DOP853 on the same open-loop C172 elevator pulse."""
from __future__ import annotations

import argparse
import csv
from dataclasses import replace
import json
from pathlib import Path
import platform
from statistics import median
from time import perf_counter

import numpy as np
from c172sim import Aircraft, __version__, level_trim, simulate_segments
from c172sim.dynamics import STATE_NAMES

# Scale unlike quantities before combining errors: 1 m/s, 1 degree,
# 1 degree/s, and 1 metre. This is a numerical comparison scale, not a limit.
ERROR_SCALES = np.array([1.0] + [np.deg2rad(1.0)] * 8 + [1.0] * 3)
METRIC_FIELDS = [
    "max_airspeed_error_m_s", "max_pitch_error_deg", "max_pitch_rate_error_deg_s",
    "max_altitude_error_m", "max_position_error_m", "max_scaled_state_error",
    "rms_scaled_state_error",
]


def error_metrics(states, reference):
    """Errors on identical timestamps; retain per-state dimensional maxima."""
    difference = states - reference
    scaled = difference / ERROR_SCALES
    return {
        "max_airspeed_error_m_s": float(np.max(np.abs(difference[:, 0]))),
        "max_pitch_error_deg": float(np.rad2deg(np.max(np.abs(difference[:, 7])))),
        "max_pitch_rate_error_deg_s": float(np.rad2deg(np.max(np.abs(difference[:, 4])))),
        "max_altitude_error_m": float(np.max(np.abs(difference[:, 11]))),
        "max_position_error_m": float(np.max(np.linalg.norm(difference[:, 9:12], axis=1))),
        "max_scaled_state_error": float(np.max(np.abs(scaled))),
        "rms_scaled_state_error": float(np.sqrt(np.mean(scaled ** 2))),
        "max_abs_error_by_state": dict(zip(STATE_NAMES, np.max(np.abs(difference), axis=0).tolist())),
    }


def matched_states(reference, times):
    """Select exact reference samples rather than interpolating a candidate."""
    indices = np.searchsorted(reference.time, times)
    if np.any(indices >= len(reference.time)) or not np.array_equal(reference.time[indices], times):
        raise RuntimeError("Reference and candidate timestamps do not match")
    return reference.states[indices]


def compare(duration, pulse_deg, steps, repeats):
    import scipy
    import matplotlib

    aircraft = Aircraft()
    trim = level_trim(aircraft)
    pulse = replace(trim.controls, elevator_rad=trim.controls.elevator_rad + np.deg2rad(pulse_deg))
    segments = [(2.0, trim.controls), (1.0, pulse), (duration - 3.0, trim.controls)]

    # Warm imports and model code once; trim, file I/O and plotting are not timed.
    simulate_segments(aircraft, trim.state, [(0.01, trim.controls)])
    rows, trajectories = [], {}
    for method in ("EULER", "RK4", "DOP853"):
        for step in steps:
            key = f"{method.lower()}_h{step:.12g}"
            row = {"run": key, "method": method, "step_limit_s": step,
                   "step_kind": "adaptive_maximum" if method == "DOP853" else "fixed",
                   "status": "failed", "elapsed_median_s": None, "nfev": None,
                   "message": "", **dict.fromkeys(METRIC_FIELDS)}
            elapsed = []
            try:
                for _ in range(repeats):
                    start = perf_counter()
                    simulation = simulate_segments(
                        aircraft, trim.state, segments, method=method,
                        step_size=step, dt=step, max_step=step)
                    elapsed.append(perf_counter() - start)
                trajectories[key] = simulation
                row.update(status="completed", elapsed_median_s=median(elapsed),
                           nfev=simulation.nfev)
            except (ValueError, RuntimeError, FloatingPointError) as error:
                row["message"] = str(error)
            rows.append(row)
            print(f"{method:7s} h={step:g} s: {row['status']}", flush=True)

    if not trajectories:
        raise RuntimeError("Every candidate failed; reduce the pulse or step sizes")
    times = np.unique(np.concatenate([sim.time for sim in trajectories.values()]))
    # DOP853's continuous interpolant evaluates the tight reference directly at
    # every candidate node; no linear interpolation of low-order results is used.
    reference = simulate_segments(
        aircraft, trim.state, segments, output_times=times,
        rtol=1e-11, atol=1e-13, max_step=0.025)
    refined = simulate_segments(
        aircraft, trim.state, segments, output_times=reference.time,
        rtol=1e-13, atol=1e-15, max_step=0.0125)
    reference_check = error_metrics(reference.states, matched_states(refined, reference.time))
    if reference_check["max_scaled_state_error"] > 1e-5:
        raise RuntimeError("Reference refinement check failed; tighten the reference settings")
    for row in rows:
        if row["status"] == "completed":
            simulation = trajectories[row["run"]]
            row.update(error_metrics(simulation.states, matched_states(reference, simulation.time)))
    report = {
        "software_version": __version__,
        "experiment": {"duration_s": duration, "elevator_increment_deg": pulse_deg,
                       "pulse_interval_s": [2, 3], "steps_s": steps, "timing_repeats": repeats},
        "environment": {"python": platform.python_version(), "numpy": np.__version__,
                        "scipy": scipy.__version__, "matplotlib": matplotlib.__version__},
        "source_commit": aircraft.data["source_commit"],
        "trim_state": dict(zip(STATE_NAMES, trim.state.tolist())),
        "reference": {"method": "DOP853", "rtol": 1e-11, "atol": 1e-13,
                      "max_step_s": 0.025, "nfev": reference.nfev,
                      "refinement_rtol": 1e-13, "refinement_atol": 1e-15,
                      "refinement_max_step_s": 0.0125,
                      "refinement_max_scaled_error_limit": 1e-5,
                      "refinement_passed": True, "refinement_difference": reference_check},
        "error_scales_in_native_units": dict(zip(STATE_NAMES, ERROR_SCALES.tolist())),
        "error_definition": "Candidate minus tight DOP853 reference at identical timestamps; maxima and RMS over saved nodes.",
        "timing_definition": "Median wall time of complete integrations, including load evaluation and output sampling; excludes trim, reference runs, file I/O and plotting. Failed runs have no timing/error score.",
        "limitations": "The reference is numerical, not an exact solution or flight-test validation. Errors near the refinement difference cannot be ranked reliably. Fixed-step domain checks include nodes and RK stages; DOP853 locates boundary events. Timing depends on hardware and execution order.",
        "runs": rows,
    }
    return report, trajectories, reference


def save_plots(output_dir, report, trajectories, reference):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    methods = ("EULER", "RK4", "DOP853")
    fig, axes = plt.subplots(3, 3, figsize=(15, 10), sharex=True, squeeze=False)
    quantities = [("Pitch angle [deg]", lambda x: np.rad2deg(x[:, 7])),
                  ("Altitude [m]", lambda x: -x[:, 11]),
                  ("True airspeed [m/s]", lambda x: x[:, 0])]
    for column, method in enumerate(methods):
        axes[0, column].set_title(method)
        for row_index, (label, extract) in enumerate(quantities):
            axis = axes[row_index, column]
            for run in report["runs"]:
                if run["method"] == method and run["status"] == "completed":
                    sim = trajectories[run["run"]]
                    step_label = "h_max" if method == "DOP853" else "h"
                    axis.plot(sim.time, extract(sim.states), label=f"{step_label}={run['step_limit_s']:g} s", lw=1.2)
            axis.plot(reference.time, extract(reference.states), "k--", lw=1.1, label="Tight reference")
            axis.axvspan(2, 3, color="orange", alpha=0.12)
            axis.set_ylabel(label)
            axis.grid(alpha=0.2)
            axis.ticklabel_format(axis="y", style="plain", useOffset=False)
        axes[0, column].legend(fontsize=8)
        axes[-1, column].set_xlabel("Time [s]")
    failed = sum(run["status"] != "completed" for run in report["runs"])
    fig.suptitle(f"Same aircraft and controls | integrator trajectories | {failed} failed runs (see report)")
    fig.tight_layout()
    fig.savefig(output_dir / "trajectories.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    definitions = [
        ("step_limit_s", "max_scaled_state_error", "Step size / adaptive maximum [s]", "Maximum scaled state error"),
        ("step_limit_s", "max_altitude_error_m", "Step size / adaptive maximum [s]", "Maximum altitude error [m]"),
        ("nfev", "max_scaled_state_error", "RHS evaluations", "Maximum scaled state error"),
        ("step_limit_s", "elapsed_median_s", "Step size / adaptive maximum [s]", "Median integration time [s]"),
    ]
    for axis, (x_key, y_key, x_label, y_label) in zip(axes.flat, definitions):
        for method in methods:
            runs = sorted((r for r in report["runs"] if r["method"] == method and r["status"] == "completed"), key=lambda r: r[x_key])
            if runs:
                axis.loglog([r[x_key] for r in runs], [max(r[y_key], 1e-16) for r in runs], "o-", label=method)
        axis.set_xlabel(x_label)
        axis.set_ylabel(y_label)
        axis.grid(which="both", alpha=0.2)
        axis.legend()
    fig.suptitle("Numerical error and cost | lower error is better | failed runs omitted")
    fig.tight_layout()
    fig.savefig(output_dir / "accuracy_and_cost.png", dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--pulse-deg", type=float, default=-0.5)
    parser.add_argument("--steps", type=float, nargs="+", default=[0.2, 0.1, 0.05, 0.025],
                        help="Fixed steps for Euler/RK4; maximum adaptive steps for DOP853 [s]")
    parser.add_argument("--repeats", type=int, default=3, help="Complete integrations per timing measurement")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "results" / "integrator_comparison")
    args = parser.parse_args()
    if (not np.isfinite(args.duration) or args.duration <= 3 or not np.isfinite(args.pulse_deg)
            or not np.all(np.isfinite(args.steps)) or min(args.steps) <= 0 or args.repeats < 1):
        parser.error("Duration must exceed 3 s; steps and repeats must be positive; numeric inputs must be finite")
    steps = sorted(set(args.steps), reverse=True)
    if len({f"{step:.12g}" for step in steps}) != len(steps):
        parser.error("Step sizes must be distinct at 12 significant digits")
    from c172sim.reporting import save_csv

    report, trajectories, reference = compare(args.duration, args.pulse_deg, steps, args.repeats)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "comparison.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    fields = ["run", "method", "step_limit_s", "step_kind", "status", "elapsed_median_s", "nfev"] + METRIC_FIELDS + ["message"]
    with (args.output_dir / "comparison.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(report["runs"])
    save_csv(args.output_dir / "reference.csv", reference)
    for key, simulation in trajectories.items():
        save_csv(args.output_dir / f"{key}.csv", simulation)
    save_plots(args.output_dir, report, trajectories, reference)
    print("\nMethod  Step [s]  Max altitude error [m]  RHS calls  Median time [s]")
    for row in report["runs"]:
        if row["status"] == "completed":
            print(f"{row['method']:7s} {row['step_limit_s']:8g} {row['max_altitude_error_m']:23.6g} {row['nfev']:10d} {row['elapsed_median_s']:16.6g}")
        else:
            print(f"{row['method']:7s} {row['step_limit_s']:8g} FAILED: {row['message']}")
    print(f"\nOutput directory: {args.output_dir}")


if __name__ == "__main__":
    main()
