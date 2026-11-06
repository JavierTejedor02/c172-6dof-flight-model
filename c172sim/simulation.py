"""Segmented flight simulation with Euler, RK4, or adaptive SciPy DOP853."""
from dataclasses import dataclass
import numpy as np
from .dynamics import rhs
from .integrators import euler_step, rk4_step

@dataclass
class Simulation:
    time: np.ndarray
    states: np.ndarray
    controls: np.ndarray
    method: str = "DOP853"
    nfev: int = 0


def segment_times(start, duration, interval):
    """Regular times plus the exact segment end; only the last gap may shorten."""
    count = int(np.ceil(duration / interval))
    offsets = np.arange(1, count, dtype=float) * interval
    # Avoid a numerically duplicated end when duration/interval is almost integer.
    offsets = offsets[offsets < duration - 8 * np.finfo(float).eps * duration]
    return np.concatenate(([start], start + offsets, [start + duration]))

def simulate_segments(aircraft, initial_state, segments, dt=0.05,
                      rtol=1e-8, atol=1e-10, max_step=0.05,
                      method="DOP853", step_size=0.05, output_times=None):
    """Integrate constant-control segments without stepping across input changes.

    Euler/RK4 save every integration node and use step_size (not dt/max_step).
    DOP853 uses dt for output spacing and max_step as its adaptive step limit.
    Optional output_times requests exact observation times from DOP853; segment
    endpoints are included too. This permits comparisons without interpolating
    the low-order trajectories. Controls at a shared boundary describe the
    segment that just ended; the next step uses the new segment's controls.
    """
    method = method.upper()
    if method not in ("EULER", "RK4", "DOP853"):
        raise ValueError("Method must be EULER, RK4, or DOP853")
    if not np.all(np.isfinite([dt, step_size])) or dt <= 0 or step_size <= 0:
        raise ValueError("Output interval and fixed step size must be finite and positive")
    if np.isnan(max_step) or max_step <= 0:
        raise ValueError("Maximum adaptive step size must be positive")
    if not np.all(np.isfinite([rtol, atol])) or rtol <= 0 or atol < 0:
        raise ValueError("Tolerances must be finite; rtol > 0 and atol >= 0")
    segments = list(segments)
    if not segments:
        raise ValueError("At least one segment is required")
    durations = np.array([duration for duration, _ in segments], dtype=float)
    if not np.all(np.isfinite(durations)) or np.any(durations <= 0):
        raise ValueError("Every segment needs a finite positive duration")
    total_duration = float(np.sum(durations))
    if not np.isfinite(total_duration):
        raise ValueError("Total duration must be finite")
    requested = None
    if output_times is not None:
        requested = np.asarray(output_times, dtype=float)
        if method != "DOP853":
            raise ValueError("Fixed-step methods return their own nodes; omit output_times")
        if (requested.ndim != 1 or requested.size == 0
                or not np.all(np.isfinite(requested))
                or np.any(np.diff(requested) <= 0)
                or requested[0] < 0 or requested[-1] > total_duration):
            raise ValueError("Output times must be finite, increasing, and inside the run")
    state = np.asarray(initial_state,dtype=float).copy()
    if state.shape != (12,) or not np.all(np.isfinite(state)):
        raise ValueError("Initial state must contain 12 finite components")
    if aircraft.domain_margin(state) <= 0:
        raise ValueError("Initial state outside the documented study domain")
    if method == "DOP853":
        from scipy.integrate import solve_ivp
    all_times,all_states,all_controls = [],[],[]
    current = 0.0
    nfev = 0
    def boundary(t,x):
        return aircraft.domain_margin(x)
    boundary.terminal=True
    boundary.direction=-1
    for duration,control in segments:
        final = current+duration
        if final <= current:
            raise ValueError("Segment duration is below floating-point time resolution")
        if method == "DOP853":
            if requested is None:
                sample = segment_times(current, duration, dt)
            else:
                inside = requested[(requested > current) & (requested < final)]
                sample = np.concatenate(([current], inside, [final]))
            result = solve_ivp(lambda t,x:rhs(t,x,aircraft,control),(current,final),state,
                               method="DOP853",t_eval=sample,events=boundary,
                               rtol=rtol,atol=atol,max_step=max_step)
            if not result.success:
                raise RuntimeError(result.message)
            if result.t_events[0].size:
                raise RuntimeError(f"Trajectory left the study domain at t={result.t_events[0][0]:.6f} s")
            times, states = result.t, result.y.T
            nfev += result.nfev
        else:
            times = segment_times(current, duration, step_size)
            states = np.empty((len(times), 12))
            states[0] = state
            step = euler_step if method == "EULER" else rk4_step

            def derivative(time, value):
                nonlocal nfev
                # Check RK stages as well as saved nodes; no event root is located.
                if not np.all(np.isfinite(value)) or aircraft.domain_margin(value) <= 0:
                    raise RuntimeError(f"{method} state/stage outside study domain at t={time:.6f} s")
                nfev += 1
                return rhs(time, value, aircraft, control)

            for index in range(len(times) - 1):
                time = times[index]
                h = times[index + 1] - time
                states[index + 1] = step(derivative, time, states[index], h)
                if (not np.all(np.isfinite(states[index + 1]))
                        or aircraft.domain_margin(states[index + 1]) <= 0):
                    raise RuntimeError(
                        f"{method} state outside study domain at t={times[index + 1]:.6f} s")
        start = 0 if not all_times else 1
        all_times.append(times[start:])
        all_states.append(states[start:])
        values=[control.elevator_rad,control.aileron_rad,control.rudder_rad,control.thrust_N]
        all_controls.append(np.tile(values,(len(times)-start,1)))
        state = states[-1]
        current = final
    return Simulation(np.concatenate(all_times),np.vstack(all_states),
                      np.vstack(all_controls),method,nfev)
