"""Piecewise-constant open-loop inputs integrated with SciPy and the model RHS."""
from dataclasses import dataclass
import numpy as np
from scipy.integrate import solve_ivp
from .dynamics import rhs

@dataclass
class Simulation:
    time: np.ndarray
    states: np.ndarray
    controls: np.ndarray

def simulate_segments(aircraft, initial_state, segments, dt=0.05,
                      rtol=1e-8,atol=1e-10,max_step=0.05):
    if dt <= 0 or max_step <= 0:
        raise ValueError("Time increments must be positive")
    state = np.asarray(initial_state,dtype=float).copy()
    if state.shape != (12,) or not np.all(np.isfinite(state)):
        raise ValueError("Initial state must contain 12 finite components")
    if aircraft.domain_margin(state) <= 0:
        raise ValueError("Initial state outside the documented study domain")
    all_times,all_states,all_controls = [],[],[]
    current = 0.0
    def boundary(t,x):
        return aircraft.domain_margin(x)
    boundary.terminal=True
    boundary.direction=-1
    for duration,control in segments:
        if duration <= 0:
            raise ValueError("Every segment needs positive duration")
        final = current+duration
        sample = np.linspace(current,final,max(1,int(np.ceil(duration/dt)))+1)
        result = solve_ivp(lambda t,x:rhs(t,x,aircraft,control),(current,final),state,
                           method="DOP853",t_eval=sample,events=boundary,
                           rtol=rtol,atol=atol,max_step=max_step)
        if not result.success:
            raise RuntimeError(result.message)
        if result.t_events[0].size:
            raise RuntimeError(f"Trajectory left the study domain at t={result.t_events[0][0]:.6f} s")
        start = 0 if not all_times else 1
        all_times.append(result.t[start:])
        all_states.append(result.y[:,start:].T)
        values=[control.elevator_rad,control.aileron_rad,control.rudder_rad,control.thrust_N]
        all_controls.append(np.tile(values,(len(result.t)-start,1)))
        state = result.y[:,-1]
        current = final
    if not all_times:
        raise ValueError("At least one segment is required")
    return Simulation(np.concatenate(all_times),np.vstack(all_states),np.vstack(all_controls))
