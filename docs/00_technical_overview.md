# Reduced C172P simulation: technical overview

## Scope

The project implements a nonlinear, rigid-body, 6-DOF civil-aircraft model, independent longitudinal/lateral pulse experiments, local perturbation matrices, candidate modal analysis and a separate quasi-steady cruise/fuel study. The twelve dynamic states use true airspeed, angle of attack and sideslip for translational velocity. Euler and RK4 provide educational fixed-step alternatives to the default adaptive DOP853 integrator. JSBSim supplies reference data and propulsion equation references; it is not executed.

The [package README](../README.md) provides prerequisites, installation, execution commands, expected outputs and verification instructions. The [equation-to-code reference](03_equations_and_code.md) defines the governing equations.

## Mass properties and load references

The loaded inertia tensor combines the base aircraft, pilot and fuel contributions, shifted to the loaded centre of gravity (CG). The liquid tanks retain the source self-inertia convention. Off-diagonal tensor entries are preserved.

Structural positions are converted to body-frame offsets relative to the loaded CG. Aerodynamic forces and thrust act at their respective reference locations; their moments include the resulting lever-arm contributions.

Density is the ISA value at 1000 m and remains constant during the examples. Fuel mass is constant in the dynamic pulse experiments. The separate performance study creates new balanced load snapshots as fuel decreases, updating the CG, tensor and moment arms.

## Architecture

| Module | Inputs | Outputs or responsibility |
|---|---|---|
| `c172sim/frames.py` | Attitude, air-data angles and body rates | Frame transformations and Euler-angle rates |
| `c172sim/aircraft.py` | State and controls | Non-gravitational resultant force and moment about the CG |
| `c172sim/dynamics.py` | State, loads and aircraft properties | Twelve state derivatives |
| `c172sim/trim.py` | Aircraft model, target speed and initial altitude | Equilibrium state and controls |
| `c172sim/integrators.py` | Derivative function, time, state and step size | Explicit Euler and classical RK4 updates |
| `c172sim/simulation.py` | Initial state and control segments | Time histories using Euler, RK4 or SciPy DOP853 |
| `run_longitudinal_demo.py` | Experiment options | CSV trajectories, JSON report and saved plots |
| `run_lateral_demo.py` | Lateral pulse options and optional recorded trim | Independent aileron/rudder responses and figures |
| `c172sim/experiments.py` | Trim controls, pulse definition and initial state | Control segments, checked trim reuse and lateral experiments |
| `c172sim/reporting.py` | Simulation time histories | Shared CSV exports |
| `compare_longitudinal_integrators.py` | Step sizes, duration and timing repeats | Numerical error/cost reports and comparison figures |
| `run_linear_analysis.py` | Checked trim, amplitudes and step size | Local matrices, candidate spectra and nonlinear comparisons |
| `c172sim/linearization.py` | Nonlinear RHS and operating condition | Smooth-part Jacobians, cusp coefficient and modal participation |
| `run_performance_analysis.py` | Speed/fuel grids and mixture policy | Quasi-steady cruise comparisons and refinement |
| `c172sim/mass_properties.py` | Remaining fuel | Independent loaded aircraft snapshots |
| `c172sim/propulsion.py` | Required thrust, axial speed and density | Matched RPM/throttle, shaft power and fuel flow |
| `c172sim/performance.py` | Candidate speeds and fuel nodes | Sampled strategy selection and integrated time/distance |

The derivative function encodes the transient physics. SciPy supplies the default equilibrium optimizer and adaptive integrator. The quasi-steady performance path uses bounded NumPy Newton trim solves, bracketed propulsion matches and fuel-coordinate quadrature. See [cruise performance](07_cruise_performance.md) for scope, equations and limitations.

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

`simulation.py` supports explicit Euler, classical RK4 and adaptive DOP853 with piecewise-constant controls. The elevator experiment is divided at input changes, and each segment starts from the preceding segment's final state. The [integrator comparison](04_integrators.md) documents the formulas, output sampling, step-size interpretation and accuracy/cost experiment.

## Reference experiment

The default experiment lasts 30 seconds. A -0.5 degree elevator increment relative to trim is applied between 2 and 3 seconds. The dashed plot trace shows the equilibrium trajectory; the blue trace shows the perturbed response.

The supplied result shows a fast response in angle of attack and pitch rate, together with a slower variation in speed and altitude. The local-model study uses linearization and eigenvalue analysis to identify candidate dynamic modes. Lateral motion is permitted by the retained CG offset, inertia products and equation coupling.

The [lateral experiments](05_lateral_experiments.md) separately apply +1 degree of effective aileron and rudder from 2 to 3 seconds, starting each trajectory from the same equilibrium. Six-panel figures show control input, sideslip, roll/yaw rates, bank and heading change. Other inputs remain at trim; no attitude or heading controller is active. The bundled RK4 results include step-refinement evidence.

The [local-model study](06_local_models_and_modes.md) compares eighteen positive/negative pulses against central and directional perturbation approximations. The `abs(beta)` drag corner is retained explicitly, not smoothed. Eight-state spectra are classified as candidate modes using modal participation; heading and position kinematics are excluded. A and B retain all twelve states for trajectory deviations.

## Verification and limitations

The test suite checks reference-frame geometry, air-data consistency, analytic gravity-only motion, torque-free invariants, physical mass properties, trim balance, equilibrium persistence, convergence, input rejection, fixed-step control boundaries, local perturbation models and quasi-steady performance. Analytic tests verify the simple integrators' expected orders. Run `python tools/verify_project.py` from the repository root. The same command runs in GitHub Actions on Windows and Linux; the [README](../README.md#verification) links to the current status and execution records.

These checks support numerical and mechanical consistency. They do not establish aerodynamic accuracy against the real aircraft. Quantitative comparison with a JSBSim execution and measured flight data remains pending.

The dynamic model omits acceleration-dependent aerodynamic terms, the additional multidimensional lift contribution, flap effects, stall hysteresis, ground effect, wind, actuators, engine/propeller dynamics, fuel burn and ground contact. The separate cruise study models stationary propulsion and changing fuel loads through successive equilibria. The [model specification](01_model_specification.md) defines the monitored study domain and control interface.
