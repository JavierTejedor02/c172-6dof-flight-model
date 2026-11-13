# Aileron and rudder pulse experiments

## Objective and initial condition

These independent open-loop experiments show the coupled lateral-directional response of the reduced C172 model. Each starts from the same straight-and-level equilibrium at 50 m/s and 1000 m. They are separate trajectories, not consecutive maneuvers on one trajectory.

The baseline retains all trim controls. Each perturbed case changes one surface and then restores that surface to trim; elevator, thrust and the other lateral surface retain their trim values throughout. The pilot's lateral CG offset means trim bank and lateral control deflections are small but nonzero.

| Parameter | Default |
|---|---|
| Duration | 30 s |
| Pulse interval | 2 to 3 s |
| Effective-aileron increment | +1 deg in the aileron experiment |
| Rudder increment | +1 deg in the rudder experiment |
| Runtime integrator | DOP853, maximum step 0.05 s |
| Bundled-result integrator | RK4, fixed step 0.01 s |

## Execution

Install dependencies using the [README](../README.md). From the repository root:

```powershell
.\.venv\Scripts\python.exe run_lateral_demo.py
```

An existing Python interpreter with the required dependencies is equally valid, for example `py -3 run_lateral_demo.py` on Windows. Run the script with no arguments using VS Code's Python Run action to use its defaults.

The default run recomputes trim. For the checked initial condition and integrator settings used in the bundled results:

```powershell
.\.venv\Scripts\python.exe run_lateral_demo.py --method RK4 --step-size 0.01 --trim-file results/longitudinal/trim_and_run.json --check-step
```

The recorded trim must match the current source revision, mass, inertia and density, lie inside the study domain, and satisfy the current equations' equilibrium acceptance checks. Otherwise it is rejected and trim must be recomputed. Reusing a recorded state does not silently override the aircraft data.

`--check-step` reruns Euler/RK4 at half the fixed step and records the shared-node differences. A failed check stops the command rather than accepting the coarse run. This option applies to fixed-step methods; DOP853 retains its own adaptive error control.

For a different experiment:

```powershell
.\.venv\Scripts\python.exe run_lateral_demo.py --aileron-deg -1 --rudder-deg 0.5 --pulse-start 2 --pulse-duration 2 --output-dir results/lateral_custom
```

Positive or negative increments are allowed within the model control limits. A zero increment is useful for a baseline check. Larger inputs may leave the study domain; a stopped trajectory is not accepted as a completed response.

## Plotted quantities

Each figure includes the prescribed control increment, sideslip beta, roll rate p, yaw rate r, bank angle phi and heading change relative to the initial heading. The dashed trace retains trim controls; the shaded interval marks the applied pulse. The command plot places the discontinuities at their prescribed times. CSV controls follow the segment-boundary storage convention documented in the [integrator guide](04_integrators.md).

Positive body axes are forward/right/down, with right-hand-positive rotations. In this model a positive effective-aileron increment initially produces positive roll acceleration. Positive rudder initially produces negative yaw acceleration, consistent with the selected source control derivatives and force-reference conventions. The effective aileron is an aerodynamic input, not an individual surface angle or normalized cockpit command.

## Interpretation

The aileron pulse produces roll as well as yaw and sideslip. Returning the aileron to trim does not command the bank angle back to its initial value. Remaining bank permits continued heading change.

The rudder pulse produces sideslip and yaw together with roll through aerodynamic and inertial coupling. A one-surface pulse is not a coordinated-turn controller. The model has no feedback loop to hold bank, heading or sideslip.

The oscillatory transients can be inspected in the plots. Formal identification of Dutch-roll, roll-subsidence or spiral modes requires linearization/eigenanalysis; these plots alone do not establish modal frequencies, damping or full-envelope stability.

## Bundled numerical results

The checked recorded equilibrium, unchanged airframe data and RK4 step of 0.01 s produced:

| Metric | Aileron pulse | Rudder pulse |
|---|---:|---:|
| Maximum absolute sideslip | 0.447685 deg | 0.801254 deg |
| Maximum absolute roll rate | 4.159962 deg/s | 1.569110 deg/s |
| Maximum absolute yaw rate | 1.313033 deg/s | 1.286646 deg/s |
| Maximum absolute bank change | 3.953401 deg | 1.279547 deg |
| Final heading change | +16.637133 deg | -3.535270 deg |

The 0.005 s refinement was compared on the shared 0.01 s observation grid. The acceptance limits were `1e-5` in native units for each of the first nine state components and `1e-3 m` for each position component. All three trajectories passed, including the baseline. Per-state measured differences are in [lateral_run.json](../results/lateral/lateral_run.json).

The largest position-component difference was approximately `3.12e-6 m`. This supports numerical consistency for these runs; it is not an error bound against an exact solution or measured flight data. The full nonlinear rigid-body model and its documented reduced aerodynamic assumptions apply throughout.

## Files and provenance

| File under `results/lateral/` | Contents |
|---|---|
| `equilibrium.csv` | Baseline with trim controls |
| `aileron.csv` | Independent aileron-pulse trajectory |
| `rudder.csv` | Independent rudder-pulse trajectory |
| `lateral_run.json` | Model, initial state, controls, solver, environment, metrics and bundled refinement evidence |
| `aileron_response.png` | Six-panel aileron response |
| `rudder_response.png` | Six-panel rudder response |

Figures are rendered from the numerical trajectories. `run_lateral_demo.py` uses Matplotlib; renderer metadata for the supplied figures is stored in `lateral_run.json`. A new run replaces files in its output directory and records its environment and settings in JSON.

## Verification

`tests/test_lateral_experiments.py` checks one-surface input isolation and restoration, initial roll/yaw control signs and coupling, independent initial states, step refinement, zero-input agreement with the baseline, and rejection of incompatible/unbalanced recorded trims.

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```
