"""Straight, level, zero-sideslip equilibrium of the reduced civil-aircraft model."""
from dataclasses import dataclass
import numpy as np
from scipy.optimize import least_squares
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

def level_trim(aircraft, airspeed=50.0, altitude=1000.0):
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
    solution = least_squares(
        residual,[0.02,0.0,0.02,0.0,0.0,900.0],
        bounds=([-0.04,-0.1,-0.3,-0.25,-0.27,0.0],
                [0.11,0.1,0.3,0.25,0.27,2500.0]),
        x_scale=[0.05,0.02,0.1,0.02,0.02,1000.0],
        ftol=1e-12,xtol=1e-12,gtol=1e-12,max_nfev=500,
    )
    x,u = unpack(solution.x)
    F,M = aircraft.loads(x,u)
    Ftotal = F + body_to_ned(*x[6:9]).T @ np.array([0,0,aircraft.mass*aircraft.gravity])
    norm = float(np.linalg.norm(residual(solution.x)))
    if not solution.success or norm > 1e-8 or aircraft.domain_margin(x) <= 0:
        raise RuntimeError(f"Trim did not converge inside domain: residual={norm}")
    if np.max(np.abs(rhs(0,x,aircraft,u)[:9])) > 1e-6:
        raise RuntimeError("Trim derivatives failed their acceptance criterion")
    return TrimResult(x,u,Ftotal,M,norm,solution.nfev)
