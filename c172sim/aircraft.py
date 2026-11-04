"""Reduced civil C172 force/moment model; all runtime units are SI."""
from dataclasses import dataclass
import json
from pathlib import Path
import numpy as np
from .frames import wind_to_body

DEFAULT_DATA = Path(__file__).resolve().parents[1] / "data" / "c172_reduced_model.json"

@dataclass(frozen=True)
class Controls:
    elevator_rad: float = 0.0
    aileron_rad: float = 0.0
    rudder_rad: float = 0.0
    thrust_N: float = 0.0

    def __post_init__(self):
        values = [self.elevator_rad,self.aileron_rad,self.rudder_rad,self.thrust_N]
        if not np.all(np.isfinite(values)) or self.thrust_N < 0:
            raise ValueError("Controls must be finite; prescribed thrust must be nonnegative")
        if abs(self.elevator_rad) > 0.34 or abs(self.aileron_rad) > 0.35 or abs(self.rudder_rad) > np.deg2rad(16):
            raise ValueError("Surface deflection outside the reduced-model interface limits")

class Aircraft:
    def __init__(self, filename=DEFAULT_DATA):
        self.data = json.loads(Path(filename).read_text(encoding="utf-8"))
        self.mass = self.data["mass_kg"]
        self.inertia = np.asarray(self.data["inertia_body_kg_m2"], dtype=float)
        self.area = self.data["wing_area_m2"]
        self.span = self.data["span_m"]
        self.chord = self.data["chord_m"]
        self.rho = self.data["density_kg_m3"]
        self.gravity = self.data["gravity_m_s2"]
        self.aero_offset = np.asarray(self.data["aero_reference_offset_body_m"])
        self.thrust_offset = np.asarray(self.data["thrust_offset_body_m"])
        self.c = self.data["coefficients"]
        self.tables = {key: np.asarray(value) for key, value in self.data["tables"].items()}
        if self.mass <= 0 or np.min(np.linalg.eigvalsh(self.inertia)) <= 0:
            raise ValueError("Invalid aircraft mass properties")

    def table(self, name, value):
        points = self.tables[name]
        # Piecewise-linear interpolation with endpoint hold is an explicit choice.
        # Some short source tables (e.g. Cl_r) do not cover the entire study domain.
        return float(np.interp(value, points[:, 0], points[:, 1]))

    def coefficients(self, state, controls):
        V, alpha, beta, p, q, r = np.asarray(state)[:6]
        if V <= 1.0:
            raise ValueError("Airspeed is outside this airborne model")
        de, da, dr = controls.elevator_rad, controls.aileron_rad, controls.rudder_rad
        phat, qhat, rhat = p*self.span/(2*V), q*self.chord/(2*V), r*self.span/(2*V)
        c = self.c
        CL = self.table("CL_static", alpha) + c["CLDe"]*de + c["CLq"]*qhat
        CD = c["CDo"] + self.table("CD_alpha", alpha) + c["CDDe"]*abs(de) + c["CDbeta"]*abs(beta)
        CY = self.table("CY_beta", beta) + c["CYda"]*da + c["CYdr"]*dr + c["CYp"]*phat + c["CYr"]*rhat
        Cl = self.table("Cl_beta", beta) + c["Clp"]*phat + self.table("Cl_r",alpha)*rhat + c["Clda"]*da + c["Cldr"]*dr
        Cm = c["Cmo"] + c["Cmalpha"]*alpha + c["Cmq"]*qhat + c["Cmde"]*de
        Cn = self.table("Cn_beta", beta) + c["Cnp"]*phat + c["Cnr"]*rhat + c["Cnda"]*da + c["Cndr"]*dr
        return np.array([CL, CD, CY, Cl, Cm, Cn])

    def loads(self, state, controls):
        """Non-gravitational resultant force and moment about the LOADED CG."""
        V, alpha, beta = np.asarray(state)[:3]
        CL, CD, CY, Cl, Cm, Cn = self.coefficients(state, controls)
        qS = 0.5*self.rho*V**2*self.area
        aero_force = wind_to_body(alpha,beta) @ (qS*np.array([-CD,CY,-CL]))
        aero_moment_reference = qS*np.array([self.span*Cl,self.chord*Cm,self.span*Cn])
        thrust_force = np.array([controls.thrust_N,0.0,0.0])
        if not np.isfinite(controls.thrust_N) or controls.thrust_N < 0:
            raise ValueError("Prescribed thrust must be finite and nonnegative")
        # M_CG = M_reference + r_reference_from_CG cross F.
        moment = (aero_moment_reference + np.cross(self.aero_offset,aero_force)
                  + np.cross(self.thrust_offset,thrust_force))
        return aero_force + thrust_force, moment

    def domain_margin(self, state):
        V, alpha, beta = np.asarray(state)[:3]
        theta = state[7]
        d = self.data["study_domain"]
        a0,a1 = np.deg2rad(d["alpha_deg"])
        return min(V-d["Va_m_s"][0], d["Va_m_s"][1]-V, alpha-a0, a1-alpha,
                   np.deg2rad(d["beta_abs_deg"])-abs(beta),
                   np.deg2rad(d["theta_abs_deg"])-abs(theta))
