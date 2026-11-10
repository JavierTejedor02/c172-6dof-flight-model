"""Local perturbation models, explicit cusp diagnostics and eight-state modes.

The full 12-state matrices describe deviations from a moving trim trajectory.
The aerodynamic CD_beta*abs(beta) term is split analytically at zero sideslip;
the physical Aircraft model is never modified or smoothed.
"""
from dataclasses import dataclass

import numpy as np
from .aircraft import Controls
from .dynamics import STATE_NAMES, rhs
from .frames import wind_to_body
from .integrators import rk4_step
from .simulation import Simulation, segment_times

INPUT_NAMES = ("elevator_rad", "aileron_rad", "rudder_rad", "thrust_N")
STATE_STEPS = np.array([1e-3] + [1e-6] * 8 + [1e-2] * 3)
INPUT_STEPS = np.array([1e-6, 1e-6, 1e-6, 0.1])
STATE_SCALES = np.array([1.0] + [np.deg2rad(1.0)] * 8 + [1.0] * 3)
DYNAMIC_INDICES = np.arange(8)
LONGITUDINAL_INDICES = np.array([0, 1, 4, 7])  # V, alpha, q, theta
LATERAL_INDICES = np.array([2, 3, 5, 6])       # beta, p, r, phi


def input_vector(controls):
    return np.array([getattr(controls, name) for name in INPUT_NAMES], dtype=float)


def controls_from_vector(values):
    return Controls(**dict(zip(INPUT_NAMES, np.asarray(values, dtype=float))))


def beta_drag_direction(aircraft, state):
    """Exact RHS coefficient multiplying abs(beta) in the selected drag term.

    Drag acts along the air-relative velocity, so it changes V but not alpha or
    beta directly. Its offset from the CG also changes angular acceleration.
    """
    speed, alpha, beta = state[:3]
    drag_per_rad = 0.5 * aircraft.rho * speed ** 2 * aircraft.area * aircraft.c["CDbeta"]
    force = wind_to_body(alpha, beta) @ np.array([-drag_per_rad, 0.0, 0.0])
    direction = np.zeros(12)
    direction[0] = -drag_per_rad / aircraft.mass
    direction[3:6] = np.linalg.solve(aircraft.inertia, np.cross(aircraft.aero_offset, force))
    return direction


def check_other_kinks(aircraft, state, controls):
    """Reject unhandled lookup/absolute-value slope corners at the trim point."""
    if abs(controls.elevator_rad) <= 1e-12 and aircraft.c["CDDe"] != 0:
        raise ValueError("Trim is at the abs(elevator) drag corner; an additional directional term is required")
    for table, value in (("CL_static", state[1]), ("CD_alpha", state[1]), ("Cl_r", state[1]),
                         ("CY_beta", state[2]), ("Cl_beta", state[2]), ("Cn_beta", state[2])):
        points = aircraft.tables[table]
        at = np.flatnonzero(np.isclose(points[:, 0], value, rtol=0, atol=1e-12))
        for index in at:
            left = ((points[index, 1] - points[index - 1, 1]) /
                    (points[index, 0] - points[index - 1, 0])) if index > 0 else 0.0
            right = ((points[index + 1, 1] - points[index, 1]) /
                     (points[index + 1, 0] - points[index, 0])) if index + 1 < len(points) else 0.0
            if not np.isclose(left, right, rtol=1e-8, atol=1e-10):
                raise ValueError(f"Trim is at an unhandled slope corner in {table}")


@dataclass
class LinearModel:
    A: np.ndarray
    B: np.ndarray
    cusp: np.ndarray
    state: np.ndarray
    controls: Controls
    diagnostics: dict

    def derivative(self, perturbation, input_change, directional=False):
        value = self.A @ perturbation + self.B @ input_change
        if directional:
            value = value + self.cusp * abs(perturbation[2])
        return value

    def nominal_state(self, time):
        nominal = self.state.copy()
        nominal[9:12] += time * np.asarray(self.diagnostics["nominal_position_rate_m_s"])
        return nominal


def linearize_aircraft(aircraft, state, controls, step_scale=1.0):
    """Central Jacobians of the smooth part plus the exact zero-beta cusp term."""
    state = np.asarray(state, dtype=float).copy()
    if state.shape != (12,) or not np.all(np.isfinite(state)) or aircraft.domain_margin(state) <= 0:
        raise ValueError("A finite, in-domain 12-state operating point is required")
    if not np.isfinite(step_scale) or step_scale <= 0:
        raise ValueError("Finite-difference step scale must be finite and positive")
    f0 = rhs(0, state, aircraft, controls)
    if np.max(np.abs(f0[:9])) > 1e-6 or abs(f0[11]) > 1e-6:
        raise ValueError("Linearization requires checked straight-and-level trim")
    # Snap only roundoff-level beta to the documented zero-sideslip condition.
    at_cusp = bool(abs(state[2]) <= 1e-12)
    if at_cusp:
        state[2] = 0.0
        f0 = rhs(0, state, aircraft, controls)
    state_steps = STATE_STEPS * step_scale
    check_other_kinks(aircraft, state, controls)
    if not at_cusp and abs(state[2]) <= state_steps[2]:
        raise ValueError("Beta finite-difference samples cross the cusp; reduce step_scale")
    input_steps = INPUT_STEPS * step_scale
    u0 = input_vector(controls)

    def smooth(value, control):
        result = rhs(0, value, aircraft, control)
        if at_cusp:
            result = result - abs(value[2]) * beta_drag_direction(aircraft, value)
        return result

    A, B = np.empty((12, 12)), np.empty((12, 4))
    for column, step in enumerate(state_steps):
        offset = np.zeros(12)
        offset[column] = step
        if aircraft.domain_margin(state + offset) <= 0 or aircraft.domain_margin(state - offset) <= 0:
            raise ValueError("Finite-difference state samples leave the study domain")
        A[:, column] = (smooth(state + offset, controls) - smooth(state - offset, controls)) / (2 * step)
    for column, step in enumerate(input_steps):
        offset = np.zeros(4)
        offset[column] = step
        B[:, column] = (smooth(state, controls_from_vector(u0 + offset))
                        - smooth(state, controls_from_vector(u0 - offset))) / (2 * step)
    step = state_steps[2]
    offset = np.zeros(12)
    offset[2] = step
    plus = (rhs(0, state + offset, aircraft, controls) - f0) / step
    minus = (f0 - rhs(0, state - offset, aircraft, controls)) / step
    cusp = beta_drag_direction(aircraft, state) if at_cusp else np.zeros(12)
    diagnostics = {
        "state_steps": state_steps.tolist(), "input_steps": input_steps.tolist(),
        "operating_point_rhs": f0.tolist(), "nominal_position_rate_m_s": f0[9:12].tolist(),
        "zero_beta_cusp": at_cusp,
        "interpretation": ("A is the central/smooth-part LTI approximation; the original RHS has no unique beta Jacobian. The directional model adds cusp*abs(delta_beta)."
                           if at_cusp else "Smooth local central-difference linearization away from beta=0."),
        "beta_derivative_positive": plus.tolist(), "beta_derivative_negative": minus.tolist(),
        "beta_derivative_gap": (plus - minus).tolist(),
        "cusp_coefficient": cusp.tolist(),
        "estimated_half_gap_minus_exact_cusp": ((plus - minus) / 2 - cusp).tolist(),
        "other_nonsmooth_features": "Piecewise-linear tables and abs(elevator) must also be checked at other operating points; the supplied trim is away from their slope discontinuities.",
    }
    return LinearModel(A, B, cusp, state, controls, diagnostics)


def matrix_refinement_check(coarse, refined):
    """Report Jacobian change after halving the differentiation increments."""
    matrices = {}
    for name in ("A", "B"):
        a, b = getattr(coarse, name), getattr(refined, name)
        relative = float(np.max(np.abs(a - b) / (1 + np.abs(b))))
        matrices[name] = {"max_abs_difference": float(np.max(np.abs(a - b))),
                          "max_difference_divided_by_one_plus_abs_refined": relative}
        if not np.isfinite(relative) or relative > 1e-5:
            raise RuntimeError(f"Finite-difference refinement failed for {name}")
    return {"passed": True, "scaled_difference_limit": 1e-5, "matrices": matrices}


def simulate_perturbations(model, segments, step_size=0.01, directional=False):
    """RK4 perturbation integration with the exact same control-switch times."""
    if not np.isfinite(step_size) or step_size <= 0:
        raise ValueError("Perturbation step must be finite and positive")
    current, state = 0.0, np.zeros(12)
    all_times, all_states, all_controls = [], [], []
    evaluations = 0
    for duration, controls in segments:
        if not np.isfinite(duration) or duration <= 0:
            raise ValueError("Segment durations must be finite and positive")
        times = segment_times(current, duration, step_size)
        values = np.empty((len(times), 12))
        values[0] = state
        change = input_vector(controls) - input_vector(model.controls)
        function = lambda t, x: model.derivative(x, change, directional)
        for index in range(len(times) - 1):
            values[index + 1] = rk4_step(function, times[index], values[index], times[index + 1] - times[index])
        if not np.all(np.isfinite(values)):
            raise RuntimeError("Perturbation trajectory is nonfinite")
        start = 0 if not all_times else 1
        all_times.append(times[start:])
        all_states.append(values[start:])
        all_controls.append(np.tile(input_vector(controls), (len(times) - start, 1)))
        evaluations += 4 * (len(times) - 1)
        current, state = times[-1], values[-1]
    if not all_times:
        raise ValueError("At least one control segment is required")
    return Simulation(np.concatenate(all_times), np.vstack(all_states), np.vstack(all_controls),
                      "RK4_DIRECTIONAL" if directional else "RK4_LTI", evaluations)


def mode_analysis(model, sector="central"):
    """Eight-state eigensystem; heading and positions are cyclic kinematics.

    Absolute biorthogonal participation products are normalized per mode.
    Labels are engineering candidates, based on participation and time scales.
    They describe this local approximation, not flight-test identification.
    """
    A = model.A.copy()
    if sector not in ("central", "positive_beta", "negative_beta"):
        raise ValueError("Unknown linearization sector")
    if sector != "central":
        A[:, 2] += model.cusp * (1 if sector == "positive_beta" else -1)
    if np.max(np.abs(A[:8, 8:])) > 1e-9:
        raise ValueError("Heading/position feed back into dynamics; the eight-state reduction is invalid")
    scales = STATE_SCALES[:8]
    dynamic = A[:8, :8]
    scaled = dynamic * scales[np.newaxis, :] / scales[:, np.newaxis]
    eigenvalues, vectors = np.linalg.eig(scaled)
    condition = float(np.linalg.cond(vectors))
    if not np.isfinite(condition) or condition > 1e10:
        raise RuntimeError("Eigenvectors are ill-conditioned; modal participation is unreliable")
    inverse = np.linalg.inv(vectors)
    modes = []
    for index, eigenvalue in enumerate(eigenvalues):
        if eigenvalue.imag < -1e-8:
            continue  # Report a conjugate pair once.
        participation = np.abs(vectors[:, index] * inverse[index, :])
        participation /= np.sum(participation)
        sigma, frequency = float(eigenvalue.real), float(abs(eigenvalue.imag))
        magnitude = float(abs(eigenvalue))
        lateral = float(np.sum(participation[LATERAL_INDICES]))
        longitudinal = float(np.sum(participation[LONGITUDINAL_INDICES]))
        modes.append({
            "eigenvalue_real_s_inv": sigma, "eigenvalue_imag_s_inv": frequency,
            "conjugate_pair": bool(frequency > 1e-8),
            "stability": "stable" if sigma < -1e-8 else "unstable" if sigma > 1e-8 else "near_neutral",
            "natural_frequency_rad_s": magnitude,
            "damping_ratio": -sigma / magnitude if magnitude > 1e-12 else None,
            "oscillation_frequency_hz": frequency / (2 * np.pi),
            "period_s": 2 * np.pi / frequency if frequency > 1e-8 else None,
            "decay_time_constant_s": -1 / sigma if sigma < -1e-8 else None,
            "growth_doubling_time_s": np.log(2) / sigma if sigma > 1e-8 else None,
            "longitudinal_participation": longitudinal, "lateral_participation": lateral,
            "participation": dict(zip(STATE_NAMES[:8], participation.tolist())),
            "candidate_label": "mixed_or_unclassified",
        })
    long_pairs = sorted([m for m in modes if m["conjugate_pair"] and m["longitudinal_participation"] > 0.7],
                        key=lambda m: m["natural_frequency_rad_s"])
    if len(long_pairs) == 2:
        long_pairs[0]["candidate_label"] = "phugoid_candidate"
        long_pairs[1]["candidate_label"] = "short_period_candidate"
    lateral_real = sorted([m for m in modes if not m["conjugate_pair"] and m["lateral_participation"] > 0.7],
                          key=lambda m: abs(m["eigenvalue_real_s_inv"]))
    if len(lateral_real) == 2:
        lateral_real[0]["candidate_label"] = "spiral_candidate"
        lateral_real[1]["candidate_label"] = "roll_subsidence_candidate"
    for mode in modes:
        if mode["conjugate_pair"] and mode["lateral_participation"] > 0.7:
            mode["candidate_label"] = "dutch_roll_candidate"
    modes.sort(key=lambda m: m["natural_frequency_rad_s"])
    return {"sector": sector, "dynamic_state_names": list(STATE_NAMES[:8]),
            "excluded_kinematic_states": list(STATE_NAMES[8:]),
            "eigenvector_condition_number_after_unit_scaling": condition,
            "participation_definition": "Absolute right-eigenvector times matching inverse-eigenvector-row entries, normalized to sum to one. Unit scaling improves conditioning; labels are candidates.",
            "limitation": "Frozen local LTI spectra; sector spectra do not prove stability of the nonsmooth nonlinear system.",
            "modes": modes}
