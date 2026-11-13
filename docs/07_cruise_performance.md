# Quasi-steady cruise, fuel burn, endurance and range

## Scope and reproduction

The performance study assembles successive fuel loads for the educational civil C172 model, balances all six forces/moments, matches stationary propeller/engine operating points and compares sampled cruise strategies. The 12-state dynamic demonstrations retain fixed mass and prescribed thrust.

With Python 3.11 or newer and the repository dependencies installed, run from the directory containing `run_longitudinal_demo.py` and `requirements.txt`:

```powershell
.\.venv\Scripts\python.exe run_performance_analysis.py
```

An existing environment can use `python run_performance_analysis.py`. The runner needs NumPy for numerical calculation and Matplotlib for ordinary plotting. `--no-plots` permits a NumPy-only numerical run. SciPy remains a dependency of the existing default trim and DOP853 demonstrations, but this sweep uses the explicit NumPy Newton trim backend.

The default study starts with the packaged half-capacity fuel load, 58.9670081 kg, and stops at 10 kg remaining. The stop amount is an illustrative retained fuel mass, not a regulation, a manufacturer reserve recommendation or a usable-fuel determination. Set `--initial-fuel-kg` and `--stop-fuel-kg` to specify other amounts within the source tank capacity.

Initial numerical settings are a 26 to 74 m/s speed grid with 1 m/s spacing and 12 equal fuel intervals. A second calculation halves both spacings. Outputs use the refined grid, and `performance.json` records the change in all three strategies' duration and distance. The acceptance limit is 1% relative change. `--skip-refinement` disables that check explicitly. The fixed-speed comparator defaults to 50 m/s; `--fixed-speed`, `--speed-min`, `--speed-max`, `--speed-step` and `--fuel-intervals` configure the experiment. Computation may take several minutes; progress is printed for each fuel load.

## One weight permits several equilibrium speeds

For nearly level unaccelerated flight, the familiar approximation is

$$
L=\tfrac12\rho V^2 S C_L\approx mg.
$$

Weight determines the required lift, but does not determine V and CL separately. A lower speed requires a larger lift coefficient, obtained with a different angle of attack and elevator trim. A higher speed requires a smaller lift coefficient. Both can be balanced flights of the same aircraft, provided that the aerodynamic, control and propulsion limits permit them.

At each speed the required resistance and motor operating point change. Lift acts approximately perpendicular to velocity; it does not itself require an equal forward thrust. Thrust balances the aerodynamic resistance along the flight path. This implementation solves the full body-axis balances, including bank, thrust inclination and reference-point moments, rather than imposing exact scalar L=mg and T=D identities.

In a conventional drag-polar explanation, parasite drag grows at high speed while lift-dependent drag becomes important at low speed. Here the actual reduced source tables determine drag, including angle-of-attack and elevator terms. They are not replaced by a fitted parabolic polar. Table corners can produce irregular local minima; the omitted source lift terms and limited alpha range further restrict physical interpretation.

## The objectives

Let q be the positive fuel mass flow, in kg/s. With no wind:

| Strategy | Local selection at each fuel load | Meaning |
|---|---|---|
| `maximum_endurance` | Smallest q among accepted speed samples | Most time per kilogram consumed |
| `maximum_range` | Smallest q/V among accepted speed samples | Most cruise distance per kilogram consumed |
| `fixed_speed` | The configured constant true airspeed | Common reference using the same fuel budget |

For example, hypothetical flights at 100 km/h and 15 kg/h or at 160 km/h and 20 kg/h illustrate the distinction. The first has lower hourly consumption, while the second has lower distance consumption: 0.125 rather than 0.15 kg/km. These illustrative numbers are not C172 results.

Minimum required power and minimum required thrust are useful textbook criteria when specific fuel consumption and propulsive efficiency are constant. Neither is imposed as the actual objective here: the source motor's internal losses and the fixed-pitch propeller's changing efficiency determine q directly. Consequently the selected endurance/range speeds need not coincide exactly with those idealized criteria.

The labels maximum refer to the accepted discrete candidates at each fuel node, not a continuous global optimum or certified aircraft performance. A selected endpoint of the feasible sampled interval is marked in each strategy CSV and the summary. Such a result can be constrained by the current trim/domain limits; performance at excluded lower speeds cannot be inferred. Failed calculations are retained separately rather than assigned invented performance values.

## Stationary propulsion and provenance

All four propulsion references use JSBSim commit `c455897049e044860b3c282e955daca1d0a785fe`, matching the aircraft source. Exact file paths, Git blob hashes, SHA-256 hashes and URLs are in `data/c172_propulsion.json`.

- `engine/eng_io320.xml`: motor configuration, including 160 hp nominal power, 2700 rpm maximum and the `bsfc` field.
- `engine/prop_75in2f.xml`: active tables for the 75-inch, two-blade, fixed-pitch propeller. Commented-out alternative tables are excluded.
- `src/models/propulsion/FGPiston.cpp`: air/fuel flow, mixture-efficiency defaults and internal engine losses.
- `src/models/propulsion/FGPropeller.cpp`: propeller coefficient conventions and tip-Mach corrections.

The Python propulsion module is an LGPL-2.1-or-later stationary adaptation with notices in [third-party information](../THIRD_PARTY_NOTICES.md) and [the included license](../LICENSES/JSBSim-LGPL.txt). It does not execute JSBSim.

For propeller rotation frequency n in revolutions per second, axial airspeed U and diameter D:

$$
J=\frac{U}{nD},\qquad T=C_T(J,M_{tip})\rho n^2D^4,
\qquad P_{shaft}=C_P(J,M_{tip})\rho n^3D^5.
$$

At trim, U=V cos(alpha) cos(beta). The runner finds an RPM whose thrust matches the aircraft requirement, then a throttle whose stationary motor power matches propeller power. Both use bracketed bisection. Source propeller table bounds, throttle bounds, 550 to 2700 rpm and the nominal 160 hp study cap are enforced. The latter is an analysis constraint, not a claim to model every manufacturer operating limit. A target without a bracket is rejected. Propulsive efficiency T U/Pshaft must lie between zero and one.

Motor flow uses source intake impedance, volumetric efficiency and the fuel/air mass ratio. The fixed default ratio is 0.07786, the lower edge of the source default mixture-efficiency plateau; its role is an explicit study choice, not a real-flight mixture instruction. It is held constant by adjusting the equivalent mixture position at the fixed atmospheric pressure. `--fuel-air-ratio` changes that policy's target. Both magnetos are assumed active. Mixture is not another optimization variable in this release.

The source field `bsfc=0.32` has units lb/(hp h), but FGPiston assigns it to indicated specific consumption. Indicated power, mixture efficiency and pumping/friction losses are evaluated separately. Applying 0.32 directly to propeller shaft power would underestimate fuel flow. The model retains those source losses and constants, including the source 287.3 J/(kg K) engine-air gas constant and its original unit-conversion constants.

Pressure and temperature are fixed ISA values at 1000 m. The airframe retains its original fixed density; sound speed uses the same atmosphere. The inlet uses isentropic total pressure with the source unit ram-air factor. Manifold-pressure lag, engine startup, cooling, propeller acceleration, gyroscopic effects, propeller reaction torque and slipstream changes to the aircraft loads are omitted. Shaft-power matching is stationary; the corresponding propeller reaction moments are not added to the existing reduced trim model.

## Depleting fuel and solving the mission

Fuel is removed equally from the source tank locations. At every fuel node the code rebuilds mass, CG, full loaded inertia, liquid self inertia and aerodynamic/thrust lever arms from the source empty aircraft, selected pilot and current tank contents. Independent Aircraft snapshots prevent a performance study from mutating the short-run configuration. Reconstruction of the original half-fuel case is tested against its packaged tensor and offsets.

Every candidate speed at every fuel node solves a new trim. The Newton backend uses the same six residual equations, search bounds and acceptance criteria as the existing SciPy trim path. It uses scaled central-difference Jacobians, bounded steps and a residual-decreasing line search. The new backend is tested against the packaged SciPy equilibrium and against force/moment/RHS balances at other loads and speeds. A failed solve is not proof that the physical aircraft cannot fly there.

For the slowly changing mission, fuel mass is a convenient independent variable:

$$
dt=-\frac{dm_f}{q(m_f,V)},\qquad
ds=-\frac{V(m_f)\,dm_f}{q(m_f,V)}.
$$

Trapezoidal quadrature over decreasing fuel nodes accumulates time and path length. This is equivalent to tracking decreasing fuel with time, while ending exactly at the selected retained amount. All strategies burn the same fuel mass. Re-selecting a speed at each node adapts to the changing weight. Under instantaneous, cost-free speed changes, pointwise selection maximizes the respective sampled integral.

This method assumes the aircraft follows the sequence of equilibria smoothly. It does not integrate acceleration between them. Taxi, takeoff, climb, descent, engine-out flight, wind and weather are absent. It does not implement variable-mass Newton-Euler dynamics, inertia-rate terms or an altitude/speed autopilot. The reported distance and time therefore describe this idealized cruise segment only.

## Outputs and verification

All outputs are written to `results/performance/`; matching files are replaced on another run. Use `--output-dir results/another_performance_study` to preserve another calculation.

| File | Contents |
|---|---|
| `operating_points.csv` | All accepted refined fuel/speed combinations, trim, power, flow, mixture and propulsion residuals |
| `rejected_points.csv` | Fuel/speed combinations and failure reasons |
| `maximum_endurance.csv`, `maximum_range.csv`, `fixed_speed.csv` | Selected nodes and accumulated mission time/distance |
| `performance.json` | Settings, provenance, assumptions, summaries, equation residuals and refinement changes |
| `summary.md` | Comparison with explicit domain-edge flags |
| `cruise_performance.png` | Initial-load angle, thrust, shaft power, hourly/distance fuel flow and efficiency versus speed |
| `cruise_missions.png` | Fuel, accumulated distance, selected speed and fuel-flow histories |

Figures are rendered from the numerical samples. The plotting command uses Matplotlib; renderer metadata for the supplied figures is recorded in JSON.

`tests/test_performance.py` checks source load reconstruction, independent snapshots, the reference equilibrium, several same-weight balanced speeds, different consumption criteria, propeller normalization, internal motor losses, inverse matching, rejected demands, analytic constant-flow mission integration and short-study exports. Run the complete suite with:

```powershell
.\.venv\Scripts\python.exe tools/verify_project.py
```

The source-derived motor and aerodynamic model have not been independently compared with measured fuel flow or the C172P performance handbook. Small numerical residuals and refined grids verify consistency of this implementation, not real-aircraft accuracy.

The [README](../README.md#verification) links to the current CI status and execution records.

## Reproduce propulsion data preparation

Download the exact raw files listed in `data/c172_propulsion.json` at the pinned revision into one directory, retaining their original filenames and bytes. Both C++ files are needed to verify the equation/default references; they are not compiled. From the repository root:

```powershell
.\.venv\Scripts\python.exe tools/build_propulsion_data.py --source-dir 'C:\path\to\pinned_propulsion_sources'
```

The command replaces `data/c172_propulsion.json`. Identity mismatches are rejected, including newline conversion. Packaged data are sufficient for ordinary execution; source downloads are optional.
