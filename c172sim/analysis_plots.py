"""Plot definitions for the local-model study; Matplotlib is imported on demand."""
import os
from pathlib import Path
import tempfile

import numpy as np
from .linearization import mode_analysis


def plot_specs(report, curves, model):
    """Return renderer-independent panels containing actual computed samples."""
    specs = []
    configurations = {
        "elevator": [(0, "Airspeed perturbation [m/s]", 1), (1, "Angle-of-attack perturbation [deg]", 180 / np.pi),
                     (4, "Pitch rate q [deg/s]", 180 / np.pi), (7, "Pitch perturbation [deg]", 180 / np.pi),
                     (2, "Sideslip beta [deg]", 180 / np.pi), (6, "Bank perturbation [deg]", 180 / np.pi)],
        "aileron": [(2, "Sideslip beta [deg]", 180 / np.pi), (3, "Roll rate p [deg/s]", 180 / np.pi),
                    (5, "Yaw rate r [deg/s]", 180 / np.pi), (6, "Bank perturbation [deg]", 180 / np.pi),
                    (0, "Airspeed perturbation [m/s]", 1), (7, "Pitch perturbation [deg]", 180 / np.pi)],
        "rudder": [(2, "Sideslip beta [deg]", 180 / np.pi), (3, "Roll rate p [deg/s]", 180 / np.pi),
                   (5, "Yaw rate r [deg/s]", 180 / np.pi), (6, "Bank perturbation [deg]", 180 / np.pi),
                   (0, "Airspeed perturbation [m/s]", 1), (7, "Pitch perturbation [deg]", 180 / np.pi)],
    }
    for surface, definitions in configurations.items():
        case = max((case for case in report["cases"] if case["surface"] == surface), key=lambda case: case["increment_deg"])
        values = curves[case["case"]]
        panels = []
        for index, label, factor in definitions:
            series = []
            for field, name, color, style in (
                ("nonlinear_delta", "Nonlinear", "#172c3b", "solid"),
                ("central_lti_delta", "Central LTI", "#1781a0", "dashed"),
                ("directional_delta", "Directional", "#d37e22", "dotted"),
            ):
                series.append({"x": values["time"], "y": values[field][:, index] * factor,
                               "label": name, "color": color, "style": style})
            panels.append({"title": label, "xlabel": "Time [s]", "ylabel": "", "series": series,
                           "shade": [2, 3], "xlim": [0, report["experiment"]["duration_s"]]})
        group = "longitudinal" if surface == "elevator" else surface
        specs.append({"filename": f"{group}_linear_nonlinear.png", "columns": 2,
                      "title": f"{surface.capitalize()} pulse | local models and nonlinear response",
                      "subtitle": f"{case['increment_deg']:+g} deg, 2-3 s | RK4 h={report['experiment']['step_size_s']:g} s | deviations from trim-control baseline",
                      "panels": panels})

    panels = []
    for surface in ("elevator", "aileron", "rudder"):
        series = []
        for approximation, label, color in (("central_lti", "Central LTI", "#1781a0"), ("directional", "Directional", "#d37e22")):
            for sign in (1, -1):
                cases = sorted((case for case in report["cases"] if case["surface"] == surface
                                and np.sign(case["increment_deg"]) == sign), key=lambda case: abs(case["increment_deg"]))
                series.append({"x": np.array([abs(case["increment_deg"]) for case in cases]),
                               "y": np.array([case[approximation]["max_dynamic_scaled_error"] / abs(case["increment_deg"]) for case in cases]),
                               "label": label + (" positive" if sign > 0 else " negative"), "color": color,
                               "style": "solid" if sign > 0 else "dashed"})
        panels.append({"title": surface.capitalize(), "xlabel": "Absolute pulse amplitude [deg]",
                       "ylabel": "Scaled dynamic error / input degree", "series": series})
    specs.append({"filename": "amplitude_sweep.png", "columns": 3,
                  "title": "Pulse-amplitude study | both input signs",
                  "subtitle": "Sampled maximum eight-state error divided by amplitude | smaller is better | scales declared in comparison.json",
                  "panels": panels})

    series = []
    for sector, label, color in (("central", "Central LTI", "#172c3b"),
                                 ("positive_beta", "Positive beta sector", "#1781a0"),
                                 ("negative_beta", "Negative beta sector", "#d37e22")):
        real, imaginary = [], []
        for mode in mode_analysis(model, sector)["modes"]:
            signs = (1, -1) if mode["conjugate_pair"] else (1,)
            for sign in signs:
                real.append(mode["eigenvalue_real_s_inv"])
                imaginary.append(sign * mode["eigenvalue_imag_s_inv"])
        series.append({"x": np.array(real), "y": np.array(imaginary), "label": label,
                       "color": color, "style": "scatter"})
    specs.append({"filename": "dynamic_modes.png", "columns": 2,
                  "title": "Frozen local dynamic spectra | eight dynamic states",
                  "subtitle": "Heading/position kinematics excluded | central and one-sided beta sectors | not a nonlinear-stability proof",
                  "panels": [
                      {"title": "Complete dynamic spectrum", "xlabel": "Real part [1/s]", "ylabel": "Imaginary part [rad/s]", "series": series},
                      {"title": "Slow-mode detail", "xlabel": "Real part [1/s]", "ylabel": "Imaginary part [rad/s]", "series": series,
                       "xlim": [-0.08, 0.005], "ylim": [-0.4, 0.4]},
                  ]})
    return specs


def render_matplotlib(output_dir, specs):
    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "c172sim-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    styles = {"solid": "-", "dashed": "--", "dotted": ":"}
    for spec in specs:
        columns = spec["columns"]
        rows = int(np.ceil(len(spec["panels"]) / columns))
        fig, axes = plt.subplots(rows, columns, figsize=(6 * columns, 3 * rows + 1), squeeze=False)
        for axis, panel in zip(axes.flat, spec["panels"]):
            for series in panel["series"]:
                if series["style"] == "scatter":
                    axis.scatter(series["x"], series["y"], color=series["color"], label=series["label"], s=28, alpha=0.7)
                else:
                    axis.plot(series["x"], series["y"], styles[series["style"]],
                              color=series["color"], label=series["label"], lw=1.5)
            if "shade" in panel:
                axis.axvspan(*panel["shade"], color="#e9a23b", alpha=0.12)
            if "xlim" in panel:
                axis.set_xlim(*panel["xlim"])
            if "ylim" in panel:
                axis.set_ylim(*panel["ylim"])
            axis.set_title(panel["title"], fontsize=10)
            axis.set_xlabel(panel["xlabel"])
            axis.set_ylabel(panel["ylabel"])
            axis.grid(alpha=0.2)
            axis.ticklabel_format(axis="y", style="sci", scilimits=(-3, 4), useOffset=False)
            axis.legend(fontsize=8, frameon=False)
        for axis in list(axes.flat)[len(spec["panels"]):]:
            axis.set_visible(False)
        fig.suptitle(spec["title"], fontsize=15, x=0.06, ha="left")
        fig.text(0.06, 0.935, spec["subtitle"], fontsize=9)
        fig.tight_layout(rect=(0, 0, 1, 0.91))
        fig.savefig(Path(output_dir) / spec["filename"], dpi=150)
        plt.close(fig)
