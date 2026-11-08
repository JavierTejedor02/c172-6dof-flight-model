"""Independent control-pulse experiments and verification of recorded trim states."""
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
from .aircraft import Controls
from .dynamics import STATE_NAMES, rhs
from .simulation import simulate_segments

SURFACE_FIELDS = {"elevator": "elevator_rad", "aileron": "aileron_rad", "rudder": "rudder_rad"}


def pulse_segments(controls, surface, increment_deg, duration=30.0,
                   pulse_start=2.0, pulse_duration=1.0):
    """Increment one surface from trim, then restore every input to trim."""
    if surface not in SURFACE_FIELDS:
        raise ValueError("Surface must be elevator, aileron, or rudder")
    if (not np.all(np.isfinite([increment_deg, duration, pulse_start, pulse_duration]))
            or pulse_start < 0 or pulse_duration <= 0
            or duration <= pulse_start + pulse_duration):
        raise ValueError("Pulse times must be finite, start >= 0, hold > 0, and finish before the run ends")
    field = SURFACE_FIELDS[surface]
    perturbed = replace(controls, **{field: getattr(controls, field) + np.deg2rad(increment_deg)})
    segments = [(pulse_duration, perturbed), (duration - pulse_start - pulse_duration, controls)]
    if pulse_start > 0:
        segments.insert(0, (pulse_start, controls))
    return segments


def checked_trim_state(aircraft, filename):
    """Read a recorded trim only if model properties and current balance agree."""
    saved = json.loads(Path(filename).read_text(encoding="utf-8"))
    if (saved["source_commit"] != aircraft.data["source_commit"]
            or not np.isclose(saved["mass_kg"], aircraft.mass, rtol=1e-12, atol=1e-10)
            or not np.isclose(saved["density_kg_m3"], aircraft.rho, rtol=1e-12, atol=1e-12)
            or np.shape(saved["inertia_body_kg_m2"]) != (3, 3)
            or not np.allclose(saved["inertia_body_kg_m2"], aircraft.inertia, rtol=1e-12, atol=1e-10)):
        raise ValueError("Recorded trim does not match the current source/mass/inertia/density; recompute trim")
    state = np.array([saved["trim_state"][name] for name in STATE_NAMES], dtype=float)
    controls = Controls(**saved["trim_controls"])
    if not np.all(np.isfinite(state)) or aircraft.domain_margin(state) <= 0:
        raise ValueError("Recorded trim state is outside the study domain")
    derivative = rhs(0, state, aircraft, controls)
    if np.max(np.abs(derivative[:9])) > 1e-6 or abs(derivative[11]) > 1e-6:
        raise ValueError("Recorded state no longer balances the current model; recompute trim")
    return state, controls


def lateral_responses(aircraft, initial_state, controls, duration=30.0,
                      aileron_deg=1.0, rudder_deg=1.0, pulse_start=2.0,
                      pulse_duration=1.0, **solver_options):
    """Return a baseline and two independent responses from the same initial state."""
    aileron = pulse_segments(controls, "aileron", aileron_deg, duration, pulse_start, pulse_duration)
    rudder = pulse_segments(controls, "rudder", rudder_deg, duration, pulse_start, pulse_duration)
    return {
        "equilibrium": simulate_segments(aircraft, initial_state, [(duration, controls)], **solver_options),
        "aileron": simulate_segments(aircraft, initial_state, aileron, **solver_options),
        "rudder": simulate_segments(aircraft, initial_state, rudder, **solver_options),
    }


def step_refinement_check(coarse_runs, fine_runs, coarse_step):
    """Compare fixed-step runs on shared nodes; accept only small state changes."""
    differences = {}
    for name, coarse in coarse_runs.items():
        fine = fine_runs[name]
        # Shared times can differ by floating-point rounding at segment edges.
        after = np.minimum(np.searchsorted(fine.time, coarse.time), len(fine.time) - 1)
        before = np.maximum(after - 1, 0)
        use_before = np.abs(fine.time[before] - coarse.time) < np.abs(fine.time[after] - coarse.time)
        indices = np.where(use_before, before, after)
        if not np.allclose(fine.time[indices], coarse.time, rtol=0, atol=1e-12):
            raise RuntimeError("Refined trajectory does not contain the coarse observation times")
        difference = np.max(np.abs(coarse.states - fine.states[indices]), axis=0)
        if (not np.all(np.isfinite(difference))
                or np.max(difference[:9]) >= 1e-5 or np.max(difference[9:]) >= 1e-3):
            raise RuntimeError(f"Step refinement failed for {name}; reduce the fixed step")
        differences[name] = difference.tolist()
    return {"coarse_step_s": coarse_step, "fine_step_s": coarse_step / 2,
            "passed": True, "max_abs_difference_per_state": differences,
            "first_nine_state_difference_limit": 1e-5,
            "position_difference_limit_m": 1e-3,
            "meaning": "Sampled numerical consistency check, not comparison with flight data."}
