"""Run independent aileron and rudder pulses and plot lateral-directional responses."""
from __future__ import annotations

import argparse
from dataclasses import asdict
from importlib.metadata import PackageNotFoundError, version
import json
import os
from pathlib import Path
import platform
import tempfile

import numpy as np
from c172sim import Aircraft, __version__, level_trim
from c172sim.dynamics import STATE_NAMES, rhs
from c172sim.experiments import checked_trim_state, lateral_responses, step_refinement_check
from c172sim.reporting import save_csv


def lateral_plot_data(surface, simulation, initial_state, experiment):
    """Six plotted quantities; command edges use the prescribed pulse times."""
    start, end = experiment["pulse_interval_s"]
    increment = experiment[f"{surface}_increment_deg"]
    command_times = np.array([0.0, start, start, end, end, experiment["duration_s"]])
    command = np.array([0.0, 0.0, increment, increment, 0.0, 0.0])
    x = simulation.states
    return [
        (f"{surface.capitalize()} increment [deg]", command_times, command),
        ("Sideslip beta [deg]", simulation.time, np.rad2deg(x[:, 2])),
        ("Roll rate p [deg/s]", simulation.time, np.rad2deg(x[:, 3])),
        ("Yaw rate r [deg/s]", simulation.time, np.rad2deg(x[:, 5])),
        ("Bank angle phi [deg]", simulation.time, np.rad2deg(x[:, 6])),
        ("Heading change [deg]", simulation.time, np.rad2deg(x[:, 8] - initial_state[8])),
    ]


def plot_response(surface, simulation, equilibrium, initial_state, experiment, filename):
    """Save a six-panel Matplotlib figure without opening a GUI window."""
    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "c172sim-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    color = "#146b8b" if surface == "aileron" else "#7851a9"
    panels = lateral_plot_data(surface, simulation, initial_state, experiment)
    baseline = lateral_plot_data(surface, equilibrium, initial_state, experiment)
    fig, axes = plt.subplots(3, 2, figsize=(12, 9))
    for index, (axis, panel, trim_panel) in enumerate(zip(axes.flat, panels, baseline)):
        label, time, values = panel
        trim_time, trim_values = trim_panel[1:]
        if index == 0:
            trim_time, trim_values = [0, experiment["duration_s"]], [0, 0]
        axis.plot(time, values, color=color, lw=1.8, label=f"{surface.capitalize()} pulse")
        axis.plot(trim_time, trim_values, color="#737e89", ls="--", lw=1.1, label="Trim controls")
        axis.axvspan(*experiment["pulse_interval_s"], color="#e9a23b", alpha=0.15)
        axis.set_ylabel(label)
        axis.set_xlim(0, experiment["duration_s"])
        axis.grid(alpha=0.2)
        axis.ticklabel_format(axis="y", style="plain", useOffset=False)
        if index >= 4:
            axis.set_xlabel("Time [s]")
    axes[0, 1].legend(frameon=False, fontsize=9)
    increment = experiment[f"{surface}_increment_deg"]
    start, end = experiment["pulse_interval_s"]
    fig.suptitle(f"Reduced C172P | open-loop {surface} response", fontsize=17, x=0.08, ha="left")
    fig.text(0.08, 0.935, f"50 m/s, 1000 m initial altitude | {increment:+g} deg from {start:g} to {end:g} s | other inputs held at trim", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(filename, dpi=160)
    plt.close(fig)


def build_report(aircraft, state, controls, responses, experiment, method, step_size, trim_source):
    """Record inputs, environment and observable response metrics."""
    derivative = rhs(0, state, aircraft, controls)
    environment = {"python": platform.python_version(), "numpy": np.__version__}
    for package in ("scipy", "matplotlib"):
        try:
            environment[package] = version(package)
        except PackageNotFoundError:
            environment[package] = None
    metrics = {}
    for surface in ("aileron", "rudder"):
        x = responses[surface].states
        metrics[surface] = {
            "max_abs_sideslip_deg": float(np.rad2deg(np.max(np.abs(x[:, 2])))),
            "max_abs_roll_rate_deg_s": float(np.rad2deg(np.max(np.abs(x[:, 3])))),
            "max_abs_yaw_rate_deg_s": float(np.rad2deg(np.max(np.abs(x[:, 5])))),
            "max_abs_bank_change_deg": float(np.rad2deg(np.max(np.abs(x[:, 6] - state[6])))),
            "final_heading_change_deg": float(np.rad2deg(x[-1, 8] - state[8])),
            "minimum_airspeed_m_s": float(np.min(x[:, 0])),
            "maximum_airspeed_m_s": float(np.max(x[:, 0])),
        }
    return {
        "software_version": __version__, "source_commit": aircraft.data["source_commit"],
        "mass_kg": aircraft.mass, "density_kg_m3": aircraft.rho,
        "inertia_body_kg_m2": aircraft.inertia.tolist(),
        "trim_source": trim_source,
        "trim_state": dict(zip(STATE_NAMES, state.tolist())), "trim_controls": asdict(controls),
        "initial_first_nine_state_derivatives": derivative[:9].tolist(),
        "initial_vertical_position_rate_m_s": float(derivative[11]),
        "experiment": experiment,
        "integration": {"method": method, "step_limit_s": step_size,
                        "step_kind": "adaptive_maximum" if method == "DOP853" else "fixed",
                        "nfev": {name: run.nfev for name, run in responses.items()}},
        "response_metrics": metrics, "environment": environment,
        "control_convention": "Actual effective-aileron and rudder deflections in model coordinates; positive signs are not a cockpit-command mapping.",
        "limitations": "Open-loop reduced-model experiments, not flight-test validation or a modal-identification result. Each pulse starts independently from the same equilibrium; no controller restores attitude or heading.",
    }


def save_results(output_dir, responses, report):
    """Write numerical results; figures may be rendered separately from these data."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    for name, run in responses.items():
        save_csv(output_dir / f"{name}.csv", run)
    (output_dir / "lateral_run.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--aileron-deg", type=float, default=1.0)
    parser.add_argument("--rudder-deg", type=float, default=1.0)
    parser.add_argument("--pulse-start", type=float, default=2.0)
    parser.add_argument("--pulse-duration", type=float, default=1.0)
    parser.add_argument("--method", choices=["DOP853", "EULER", "RK4"], default="DOP853")
    parser.add_argument("--step-size", type=float, default=0.05,
                        help="Fixed step for Euler/RK4; maximum adaptive step for DOP853 [s]")
    parser.add_argument("--trim-file", type=Path, help="Reuse a recorded equilibrium after checking model properties and balance")
    parser.add_argument("--check-step", action="store_true",
                        help="Verify Euler/RK4 trajectories against a second run with half the fixed step")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "results" / "lateral")
    args = parser.parse_args()
    if (not np.all(np.isfinite([args.duration, args.aileron_deg, args.rudder_deg,
                               args.pulse_start, args.pulse_duration, args.step_size]))
            or args.step_size <= 0 or args.pulse_start < 0 or args.pulse_duration <= 0
            or args.duration <= args.pulse_start + args.pulse_duration):
        parser.error("Inputs must be finite; step and pulse duration positive; pulse start >= 0; run must extend beyond the pulse")
    if args.check_step and args.method == "DOP853":
        parser.error("--check-step applies to fixed-step methods; select --method RK4 or EULER")
    aircraft = Aircraft()
    if args.trim_file is None:
        trim = level_trim(aircraft)
        state, controls = trim.state, trim.controls
        trim_source = "Recomputed straight-and-level equilibrium"
    else:
        state, controls = checked_trim_state(aircraft, args.trim_file)
        trim_source = f"Checked recorded equilibrium: {args.trim_file.name}"
    experiment = {"duration_s": args.duration, "aileron_increment_deg": args.aileron_deg,
                  "rudder_increment_deg": args.rudder_deg,
                  "pulse_interval_s": [args.pulse_start, args.pulse_start + args.pulse_duration]}
    responses = lateral_responses(
        aircraft, state, controls, args.duration, args.aileron_deg, args.rudder_deg,
        args.pulse_start, args.pulse_duration, method=args.method,
        step_size=args.step_size, max_step=args.step_size)
    report = build_report(aircraft, state, controls, responses, experiment,
                          args.method, args.step_size, trim_source)
    if args.check_step:
        refined = lateral_responses(
            aircraft, state, controls, args.duration, args.aileron_deg, args.rudder_deg,
            args.pulse_start, args.pulse_duration, method=args.method,
            step_size=args.step_size / 2, max_step=args.step_size / 2)
        report["step_refinement_check"] = step_refinement_check(responses, refined, args.step_size)
    report["figure_renderer"] = "Matplotlib"
    save_results(args.output_dir, responses, report)
    for surface in ("aileron", "rudder"):
        plot_response(surface, responses[surface], responses["equilibrium"], state,
                      experiment, args.output_dir / f"{surface}_response.png")
    print(json.dumps({"output_dir": str(args.output_dir), "response_metrics": report["response_metrics"]}, indent=2))


if __name__ == "__main__":
    main()
