"""Newton-Euler dynamics and Euler-angle kinematics for the reduced C172 model."""
import numpy as np
from .frames import body_velocity, body_to_ned, euler_rates

STATE_NAMES = ("Va_m_s","alpha_rad","beta_rad","p_rad_s","q_rad_s","r_rad_s",
               "phi_rad","theta_rad","psi_rad","xN_m","xE_m","xD_m")

def rhs_from_loads(state, force_body, moment_body, mass, inertia, gravity=9.80665):
    """Fixed mass/inertia, flat Earth, no wind. force_body excludes gravity."""
    x = np.asarray(state, dtype=float)
    V, alpha, beta = x[:3]
    omega, attitude = x[3:6], x[6:9]
    if V <= 1.0 or abs(np.cos(beta)) < 1e-3:
        raise ValueError("Air-data state representation is near a singularity")
    velocity = body_velocity(V,alpha,beta)
    Cbn = body_to_ned(*attitude)
    gravity_body = Cbn.T @ np.array([0.0,0.0,gravity])
    acceleration_body = np.asarray(force_body)/mass + gravity_body - np.cross(omega,velocity)

    u,v,w = velocity
    udot,vdot,wdot = acceleration_body
    Vdot = np.dot(velocity,acceleration_body)/V
    alphadot = (u*wdot-w*udot)/(u*u+w*w)
    betadot = (vdot - (v/V)*Vdot)/(V*np.cos(beta))
    omegadot = np.linalg.solve(inertia, np.asarray(moment_body)-np.cross(omega,inertia@omega))
    return np.concatenate(([Vdot,alphadot,betadot],omegadot,
                           euler_rates(attitude[0],attitude[1],omega),
                           Cbn@velocity))

def rhs(t, state, aircraft, controls):
    force, moment = aircraft.loads(state,controls)
    return rhs_from_loads(state,force,moment,aircraft.mass,aircraft.inertia,aircraft.gravity)
