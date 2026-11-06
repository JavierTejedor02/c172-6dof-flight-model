"""Educational fixed-step formulas for a first-order system dx/dt = fun(t, x)."""


def euler_step(fun, time, state, step_size):
    """Advance all state components using the slope at the start of the step."""
    derivative = fun(time, state)
    next_state = state + step_size * derivative
    return next_state


def rk4_step(fun, time, state, step_size):
    """Classical fourth-order Runge-Kutta: four slopes and a weighted average."""
    k1 = fun(time, state)
    k2 = fun(time + step_size / 2, state + step_size * k1 / 2)
    k3 = fun(time + step_size / 2, state + step_size * k2 / 2)
    k4 = fun(time + step_size, state + step_size * k3)
    next_state = state + step_size * (k1 + 2 * k2 + 2 * k3 + k4) / 6
    return next_state
