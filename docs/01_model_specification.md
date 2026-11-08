# Civil C172 model specification

Status: version 0.3.0 includes independent longitudinal/lateral pulse experiments, fixed-step integrators and an integrator comparison.

## Purpose and reference

Educational reduced Cessna 172P model based on JSBSim `aircraft/c172x/c172x.xml` at revision `c455897049e044860b3c282e955daca1d0a785fe`. The Python implementation solves the nonlinear rigid-body equations. JSBSim is not a runtime dependency.

The source represents a 1982 C172P and includes estimates as well as public data. The reduced model is not a full reproduction of that source.

## State and references

State order: `[Va, alpha, beta, p, q, r, phi, theta, psi, xN, xE, xD]`.

SI units throughout: metres, seconds, kilograms, newtons, radians. Body axes are forward/right/down, origin at loaded CG; NED is a fixed local frame; Euler sequence is 3-2-1. Alpha is body-axis angle of attack and beta is positive for positive lateral air-relative body velocity.

The source structural frame is aft/right/up with its own origin. Structural positions are converted to body-frame displacements from the loaded CG before computing force moments and inertia shifts.

Forces are first described in wind axes as drag, side force and lift, transformed to body axes, and applied at the source aerodynamic reference point. Native aerodynamic moments are represented in body axes. Prescribed thrust is axial and applied at the source propeller location.

## Mass and atmosphere

`data/load_case_v0.json` specifies one pilot, half fuel in each tank and other point masses zero. Mass and inertia are constant during a run. Loaded inertia includes parallel-axis contributions and the source isotropic self inertia of the two liquid tanks.

`data/c172_reduced_model.json` is the runtime source of geometry, loaded properties, selected aerodynamic data and reference offsets. Density is 1.1116425003060326 kg/m^3, the ISA value at 1000 m, held constant; gravity is 9.80665 m/s^2; no wind or Earth rotation is modelled.

## Inputs and domain

Inputs: elevator, effective aileron, rudder in radians, and prescribed nonnegative axial thrust in newtons. Cockpit normalized-command mapping and actuator dynamics are absent. Thrust is not a calibrated engine/propeller prediction.

Surface input bounds: absolute elevator <= 0.34 rad; effective aileron <= 0.35 rad; rudder <= 16 deg. The trim search uses narrower numerical bounds. These interface bounds and the following study domain are modelling constraints, not certified aircraft limits.

Monitored study domain: 25 < Va < 75 m/s, -2.5 < alpha < 6.5 deg, absolute beta < 8 deg, absolute pitch < 60 deg. An accepted initial state must lie strictly inside; integration stops if a boundary is reached. Coefficient lookup uses linear interpolation and endpoint hold outside a table range, including short tables such as Cl_r. The accepted trajectory is separately constrained to the study domain; it is not guaranteed to remain inside every individual table range.

## Aerodynamic scope

Retain the un-stalled static CL table, zero-flap CD table, selected elevator and lateral-control terms, and p/q/r damping terms. Retain the source beta tables and alpha-dependent yaw-rate roll derivative.

Omit CLadot and Cmadot, additional multidimensional CLalpha, flap effects, stall hysteresis, ground effect, propulsion/slipstream/gyroscopic effects, fuel burn and ground contact. The source term inventory and runnable reduced-model JSON identify the omissions.

## Example and verification

The demonstration solves straight, level, beta=0 flight at 50 m/s and 1000 m. Small lateral trim offsets are allowed for the left-seat pilot's CG displacement. The solver balances all three force components and all three moment components; theta enforces zero vertical velocity.

The original version 0.1.0 results contain two 30 s runs: steady trim and a -0.5 deg elevator increment between t=2 and t=3 s. DOP853 remains the default, with a maximum adaptive step of 0.05 s. Version 0.2.0 also supports explicit Euler and classical RK4, with a configurable fixed step. Every method splits intervals at input changes. Fixed methods save each step and shorten the last step of a segment; DOP853 separates output spacing from its adaptive internal steps.

The suite contains twenty-six tests covering frames, analytic mechanics, mass/inertia, trim, equilibrium persistence, convergence, invalid inputs, fixed-step control boundaries, comparison metrics and lateral experiments. The original ten-test result file is historical evidence, not a run of the expanded suite. Measured flight-data validation and quantitative JSBSim comparison remain pending.

`compare_integrators.py` compares the perturbed trajectory using Euler, RK4 and DOP853, reports sampled numerical errors against a tight and refined DOP853 reference, and measures RHS evaluations and elapsed integration time. Reference convergence checks, failure handling and metric scales are described in [integrators and comparison](04_integrators.md).

`run_lateral_demo.py` adds independent +1 degree effective-aileron and rudder pulses, each held from 2 to 3 seconds in a 30 s run. Only the selected surface changes; the remaining controls stay at trim. The bundled results use checked recorded trim and RK4 with 0.01 s steps, verified against 0.005 s steps. Signs, metrics, outputs and limitations are documented in [lateral experiments](05_lateral_experiments.md).
