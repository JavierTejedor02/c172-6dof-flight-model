"""Frame conventions: body x forward/y right/z down; local NED; Euler 3-2-1."""
import numpy as np

def body_velocity(Va, alpha, beta):
    """Air-relative velocity components, expressed in body axes."""
    return Va * np.array([np.cos(alpha)*np.cos(beta), np.sin(beta),
                          np.sin(alpha)*np.cos(beta)])

def body_to_ned(phi, theta, psi):
    """Active direction-cosine matrix: v_ned = C_body_to_ned @ v_body."""
    sp, cp = np.sin(phi), np.cos(phi)
    st, ct = np.sin(theta), np.cos(theta)
    ss, cs = np.sin(psi), np.cos(psi)
    return np.array([
        [ct*cs, sp*st*cs-cp*ss, cp*st*cs+sp*ss],
        [ct*ss, sp*st*ss+cp*cs, cp*st*ss-sp*cs],
        [-st, sp*ct, cp*ct],
    ])

def wind_to_body(alpha, beta):
    """Transforms wind-axis force [-D, Y, -L] to body axes."""
    ca, sa = np.cos(alpha), np.sin(alpha)
    cb, sb = np.cos(beta), np.sin(beta)
    return np.array([[ca*cb, -ca*sb, -sa],
                     [sb, cb, 0.0],
                     [sa*cb, -sa*sb, ca]])

def euler_rates(phi, theta, omega):
    """Euler-angle derivatives from body angular rates p, q, r."""
    p, q, r = omega
    cp, sp, ct = np.cos(phi), np.sin(phi), np.cos(theta)
    if abs(ct) < 1e-3:
        raise ValueError("Euler 3-2-1 representation is near its pitch singularity")
    return np.array([p + np.tan(theta)*(q*sp+r*cp),
                     q*cp-r*sp, (q*sp+r*cp)/ct])
