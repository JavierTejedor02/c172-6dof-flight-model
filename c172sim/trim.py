"""Straight, level, zero-sideslip equilibrium of the reduced civil-aircraft model."""
from dataclasses import dataclass
import numpy as np
from .aircraft import Controls
from .frames import body_to_ned
from .dynamics import rhs

@dataclass
class TrimResult:
    state: np.ndarray
    controls: Controls
    force_residual_N: np.ndarray
    moment_residual_Nm: np.ndarray
    scaled_residual_norm: float
    evaluations: int

def level_trim(aircraft, airspeed=50.0, altitude=1000.0, solver="scipy"):
    """Solve six force/moment balances; optional NumPy Newton backend for sweeps."""
    if not 25 < airspeed < 75:
        raise ValueError("Requested trim speed outside the study domain")
    # Unknowns: alpha, bank, elevator, effective aileron, rudder, axial thrust.
    # beta=0 and p=q=r=0. theta enforces exactly zero vertical ground velocity.
    def unpack(z):
        alpha,phi,de,da,dr,thrust = z
        theta = np.arctan(np.cos(phi)*np.tan(alpha))
        x = np.array([airspeed,alpha,0,0,0,0,phi,theta,0,0,0,-altitude],dtype=float)
        return x, Controls(de,da,dr,thrust)
    def residual(z):
        x,u = unpack(z)
        F,M = aircraft.loads(x,u)
        gravity_body = body_to_ned(*x[6:9]).T @ np.array([0,0,aircraft.mass*aircraft.gravity])
        weight = aircraft.mass*aircraft.gravity
        return np.concatenate(((F+gravity_body)/weight,
                               M/(weight*np.array([aircraft.span,aircraft.chord,aircraft.span]))))
    initial = np.array([0.02,0.0,0.02,0.0,0.0,900.0])
    lower = np.array([-0.04,-0.1,-0.3,-0.25,-0.27,0.0])
    upper = np.array([0.11,0.1,0.3,0.25,0.27,2500.0])
    scales = np.array([0.05,0.02,0.1,0.02,0.02,1000.0])
    if solver == "scipy":
        from scipy.optimize import least_squares
        solution = least_squares(residual, initial, bounds=(lower, upper), x_scale=scales,
                                 ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=500)
        z, success, evaluations = solution.x, solution.success, solution.nfev
    elif solver == "newton":
        z, success, evaluations = _bounded_newton(residual, initial, lower, upper, scales)
    else:
        raise ValueError("Trim solver must be scipy or newton")
    x,u = unpack(z)
    F,M = aircraft.loads(x,u)
    Ftotal = F + body_to_ned(*x[6:9]).T @ np.array([0,0,aircraft.mass*aircraft.gravity])
    norm = float(np.linalg.norm(residual(z)))
    if not success or norm > 1e-8 or aircraft.domain_margin(x) <= 0:
        raise RuntimeError(f"Trim did not converge inside domain: residual={norm}")
    if np.max(np.abs(rhs(0,x,aircraft,u)[:9])) > 1e-6:
        raise RuntimeError("Trim derivatives failed their acceptance criterion")
    return TrimResult(x,u,Ftotal,M,norm,evaluations)


def _bounded_newton(function, initial, lower, upper, scales):
    """Finite-difference Newton steps with bounds and a residual-decreasing search.

    This backend is intended for the regular steady-flight branch. Failed
    points are rejected; all existing trim acceptance checks still apply.
    """
    y, lo, hi = initial / scales, lower / scales, upper / scales
    calls = 0

    def evaluate(value):
        nonlocal calls
        calls += 1
        return function(value * scales)

    for _ in range(35):
        f = evaluate(y)
        if np.linalg.norm(f) < 1e-11:
            return y * scales, True, calls
        jacobian = np.empty((6, 6))
        for index in range(6):
            plus, minus = y.copy(), y.copy()
            plus[index] = min(y[index] + 1e-5, hi[index])
            minus[index] = max(y[index] - 1e-5, lo[index])
            jacobian[:, index] = (evaluate(plus) - evaluate(minus)) / (plus[index] - minus[index])
        try:
            direction = np.linalg.solve(jacobian, -f)
        except np.linalg.LinAlgError:
            break
        for power in range(18):
            candidate = np.clip(y + direction * 0.5 ** power, lo, hi)
            if np.linalg.norm(evaluate(candidate)) < np.linalg.norm(f):
                y = candidate
                break
        else:
            break
    return y * scales, False, calls
