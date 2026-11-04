# Reduced C172P simulation: technical overview

## Scope

Version 0.1.0 implements a nonlinear, rigid-body, 6-DOF civil-aircraft model. It solves a straight-and-level equilibrium and integrates an equilibrium trajectory and an open-loop elevator-pulse response. The twelve states use true airspeed, angle of attack and sideslip for translational velocity. JSBSim supplies reference data and is not executed.

The [package README](../README.md) provides prerequisites, installation, execution commands, expected outputs and verification instructions. The [equation-to-code reference](03_equations_and_code.md) defines the governing equations.

## Mass properties and load references

The loaded inertia tensor combines the base aircraft, pilot and fuel contributions, shifted to the loaded centre of gravity (CG). The liquid tanks retain the source self-inertia convention. Off-diagonal tensor entries are preserved.

Structural positions are converted to body-frame offsets relative to the loaded CG. Aerodynamic forces and thrust act at their respective reference locations; their moments include the resulting lever-arm contributions.

Density is the ISA value at 1000 m and remains constant during the examples. Fuel mass is also constant.

## Architecture

| Module | Inputs | Outputs or responsibility |
|---|---|---|
| `c172sim/frames.py` | Attitude, air-data angles and body rates | Frame transformations and Euler-angle rates |
| `c172sim/aircraft.py` | State and controls | Non-gravitational resultant force and moment about the CG |
| `c172sim/dynamics.py` | State, loads and aircraft properties | Twelve state derivatives |
| `c172sim/trim.py` | Aircraft model, target speed and initial altitude | Equilibrium state and controls |
| `c172sim/simulation.py` | Initial state and control segments | Time histories using SciPy DOP853 |
| `run_demo.py` | Experiment options | CSV trajectories, JSON report and saved plots |

The derivative function encodes the physics. SciPy supplies the numerical algorithms for equilibrium optimization and time integration.

## Implementation details

### Reference frames

`frames.py` defines wind-to-body and body-to-NED transformations. The first column of the wind-to-body matrix follows the air-relative velocity direction. Under the no-wind assumption, the body-to-NED transformation maps that velocity to the ground-relative position rate.

### Equations of motion

`dynamics.py` reconstructs auxiliary body velocity components u, v and w, evaluates their Newton-equation derivatives, then maps those derivatives to true airspeed, angle of attack and sideslip rates. Euler's rigid-body equation supplies angular acceleration. Attitude and position kinematics complete the twelve-state derivative. The auxiliary components do not change the integrated state definition.

### Aerodynamic and thrust loads

`aircraft.py` evaluates selected source tables, control terms and rate derivatives. Moments are shifted from their application points to the loaded CG. The [data audit](02_data_audit.md) documents the retained terms and omissions. Axial thrust is prescribed in newtons; engine, propeller and throttle dynamics are absent.

### Equilibrium

`trim.py` solves for angle of attack, bank, elevator, effective aileron, rudder and thrust, with sideslip and body rates fixed at zero. Pitch attitude enforces zero vertical ground velocity. All three total force components and all three moment components are balanced. Small lateral offsets are allowed because of the left-seat pilot's CG displacement.

The demonstration uses 50 m/s and an initial altitude of 1000 m. Changing the trim function's altitude argument changes the initial NED position; it does not recalculate the constant density.

### Time integration

`simulation.py` uses DOP853 with piecewise-constant controls. The elevator experiment is divided at the input changes so the integrator does not step across a discontinuity as though it were smooth. Each segment starts from the preceding segment's final state.

## Reference experiment

The default experiment lasts 30 seconds. A -0.5 degree elevator increment relative to trim is applied between 2 and 3 seconds. The dashed plot trace shows the equilibrium trajectory; the blue trace shows the perturbed response.

The supplied result shows a fast response in angle of attack and pitch rate, together with a slower variation in speed and altitude. Formal modal identification would require linearization and eigenvalue analysis, which are outside this release. Lateral motion is permitted by the retained CG offset, inertia products and equation coupling.

## Verification and limitations

The test suite checks reference-frame geometry, air-data consistency, analytic gravity-only motion, torque-free energy and inertial angular momentum conservation, physical mass properties, trim balance, equilibrium persistence, solver convergence and zero-speed rejection. The packaged record reports ten passing tests. Reproduction commands are in the README.

These checks support numerical and mechanical consistency. They do not establish aerodynamic accuracy against the real aircraft. Quantitative comparison with a JSBSim execution and measured flight data remains pending.

The reduced model omits acceleration-dependent aerodynamic terms, the additional multidimensional lift contribution, flap effects, stall hysteresis, ground effect, wind, actuators, engine/propeller dynamics, fuel burn and ground contact. The [model specification](01_model_specification.md) defines the monitored study domain and control interface.
