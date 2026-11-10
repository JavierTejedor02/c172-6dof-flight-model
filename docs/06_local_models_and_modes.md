# Local perturbation models, candidate modes and nonlinear comparison

## Scope and reproduction

Version 0.4.0 studies the existing reduced civil C172 model near its 50 m/s, 1000 m straight-and-level trim. It exports full twelve-state perturbation matrices, analyzes eight dynamic states and compares positive/negative control pulses of 0.25, 0.5 and 1 degree. The original aerodynamic and rigid-body equations remain unchanged.

From the repository root with dependencies installed:

```powershell
.\.venv\Scripts\python.exe run_linear_analysis.py
```

An existing interpreter can be used, for example `py -3 run_linear_analysis.py`. The default run recomputes trim. To reproduce the checked recorded trim used by the bundled results:

```powershell
.\.venv\Scripts\python.exe run_linear_analysis.py --trim-file results/longitudinal/trim_and_run.json
```

All comparisons use RK4 with a 0.01 s step, split at the 2 and 3 s pulse edges. By default the positive largest-amplitude case for each surface is repeated with a 0.005 s step. `--skip-step-check` explicitly disables this check. `--duration`, `--amplitudes`, `--step-size` and `--output-dir` select another study. `--no-plots` writes numerical outputs without importing Matplotlib; with a recorded trim, this path needs only NumPy.

## Moving trim and perturbation coordinates

The state is:

$$
x=[V,\alpha,\beta,p,q,r,\phi,\theta,\psi,x_N,x_E,x_D]^T.
$$

The inputs are actual elevator, effective aileron and rudder deflections in radians, followed by prescribed axial thrust in newtons. The operating condition has constant velocity and attitude, but its position advances. It is not a fixed point of all twelve states.

Define deviations from the straight-moving nominal trajectory and its constant trim controls:

$$
\delta x=x-x_0(t),\qquad \delta u=u-u_0.
$$

For the current constant-density, no-wind, flat-Earth assumptions, heading and position do not feed back into the first eight state derivatives. The full A and B include their kinematics, but the mode analysis uses only `[V, alpha, beta, p, q, r, phi, theta]`. Four neutral heading/position coordinates are therefore not misidentified as aircraft dynamic modes. The code rejects this reduction if future model changes introduce such feedback.

## The zero-sideslip drag corner

`aircraft.py` contains the selected source term `CDbeta * abs(beta)`, with `CDbeta = 0.17`. At trim, beta is zero. The left and right derivatives differ: the original RHS has no unique ordinary beta Jacobian there.

For this particular term the existing RHS can be decomposed exactly as:

$$
f(x,u)=f_s(x,u)+g(x)|\beta|.
$$

The drag force is parallel to air-relative velocity, so this term contributes directly to the airspeed derivative. Its reference-point offset also contributes angular acceleration. `beta_drag_direction` computes that contribution, including the full inertia tensor. It is tested against the original force/moment implementation with this term isolated; the production aircraft data are not changed.

Two approximations are exported and simulated:

$$
\begin{aligned}
\text{Central LTI:}\quad &\delta\dot x=A\delta x+B\delta u,\\
\text{Directional:}\quad &\delta\dot x=A\delta x+B\delta u+g_0|\delta\beta|.
\end{aligned}
$$

A is the Jacobian of the smooth part at zero sideslip; B is the input Jacobian. A is the limiting central-average approximation of the original RHS, not a unique Jacobian of that nonsmooth RHS. The directional approximation is piecewise linear and is not an LTI system. Its frozen sector matrices are:

$$
A_+=A+g_0e_\beta^T,\qquad A_-=A-g_0e_\beta^T.
$$

The study reports both original one-sided beta derivatives and their gap. It also rejects an unhandled table-slope corner or zero-elevator absolute-value corner at another trim, rather than silently treating that point as smooth. The default trim's other lookup slopes are locally continuous.

## Numerical differentiation

Readable central differences are implemented in `c172sim/linearization.py`. Base state increments are 0.001 m/s for airspeed, 1e-6 rad or rad/s for angular states, and 0.01 m for positions. Base input increments are 1e-6 rad for surfaces and 0.1 N for thrust.

The matrices used for the report use half those increments and are checked against the base increments. The acceptance metric is the largest componentwise `abs(coarse-refined)/(1+abs(refined))`, in the declared native units, below 1e-5. It is a numerical consistency criterion, not an independent accuracy guarantee. The bundled maximum is approximately 3.55e-9 for A and 1.02e-11 for B.

## Modal analysis

The central and both frozen-sector eight-state matrices are analyzed separately. Eigenvalues, natural frequency, oscillation period, damping ratio and stable decay/unstable growth times are reported. A conjugate pair is listed once with a positive imaginary component.

Modal participation uses the absolute product of each right-eigenvector component and the corresponding inverse-eigenvector-row component, normalized within the mode. Unit scaling improves conditioning; this participation definition is invariant under state-unit scaling and is not an energy fraction.

Candidate labels combine longitudinal/lateral participation and relative time scale. They are inspectable engineering labels, not guaranteed real-aircraft mode identification. The full coupled matrices are retained; longitudinal and lateral blocks are not forced to decouple despite the asymmetric load case.

For the bundled central approximation:

| Candidate | Eigenvalue [1/s] | Damping ratio | Period [s] |
|---|---|---:|---:|
| Spiral | -0.0157762 | 1.000 | - |
| Phugoid | -0.0212799 +/- 0.240459j | 0.0882 | 26.130 |
| Dutch roll | -0.490168 +/- 2.43513j | 0.1973 | 2.580 |
| Short period | -3.18888 +/- 5.98045j | 0.4705 | 1.051 |
| Roll subsidence | -7.08911 | 1.000 | - |

These central eigenvalues have negative real parts. The central and sector spectra do not by themselves prove stability of the nonsmooth nonlinear model. A real eigenvalue's reported damping ratio of 1 does not imply an oscillation; its decay time constant is the useful time measure. Detailed participation, conditioning and sector results are in [modes.json](../results/linear_analysis/modes.json) and [modes.md](../results/linear_analysis/modes.md).

## Pulse comparisons and error metrics

Every pulse is an increment about the same trim, held from 2 to 3 s. The nonlinear perturbed trajectory is compared with the common nonlinear trim-control baseline at identical timestamps. The approximations start from zero perturbation and use identical inputs and segment boundaries. The comparison concerns perturbations, including position deviations, rather than comparing absolute positions to a static trim point.

The amplitude sweep contains eighteen cases: three surfaces, three amplitudes and both signs. Maximum and RMS dynamic errors scale the first eight state components by 1 m/s, 1 degree or 1 degree/s. These are comparison scales, not accuracy requirements. Relative dynamic error divides the maximum scaled error by the peak scaled nonlinear dynamic response. It is not a percentage error for every individual state. Dimensional errors for every state and altitude are retained separately.

The figures select the positive largest-amplitude case for each control. Lateral figures include airspeed and pitch to expose the cross-axis influence of the drag corner. The amplitude figure divides maximum scaled dynamic error by input amplitude; maxima are over saved samples, not guaranteed continuous-time bounds.

For a +0.25 degree rudder pulse, the bundled relative dynamic error is approximately 10.04% for the central approximation and 0.33% for the directional approximation. The central error floor at small lateral inputs is consistent with its omitted first-order absolute-beta term. The directional approximation still has nonlinear remainder errors; it need not improve every metric in every longitudinal case.

## Integration refinement and limits

The three positive largest-amplitude comparisons are repeated with half the RK4 step. The nonlinear, central and directional traces must differ by less than 1e-5 in each of the first nine native state components and 1e-3 m in each position component at shared nodes. Actual differences are recorded in `comparison.json`. The check covers those three representative inputs, not every possible maneuver.

The original reduced-aerodynamic limitations apply. These checks establish internal numerical consistency, not agreement with JSBSim execution, measured C172 flight data or certified operating limits. A finite-difference matrix and recognizable mode names do not constitute aircraft validation.

## Output files

All study outputs are under `results/linear_analysis/`:

| File | Contents |
|---|---|
| `A.csv`, `B.csv` | Ordered full twelve-state central/smooth-part matrices with row/column labels |
| `A_positive_beta.csv`, `A_negative_beta.csv` | Frozen one-sided sector matrices |
| `linearization.json` | Matrices, units/order, trim, cusp vector, one-sided derivatives and differentiation check |
| `modes.json`, `modes.md` | Central/sector spectra, candidate labels and participation |
| `comparison.json`, `amplitude_sweep.csv` | Settings, metrics for all eighteen cases and integration evidence |
| `perturbation_trajectories.npz` | All time histories; numeric/string arrays load with `allow_pickle=False` |
| `elevator_comparison.csv`, `aileron_comparison.csv`, `rudder_comparison.csv` | Full perturbation traces for the positive largest-amplitude cases |
| `longitudinal_linear_nonlinear.png`, `aileron_linear_nonlinear.png`, `rudder_linear_nonlinear.png` | Six-panel comparisons |
| `amplitude_sweep.png`, `dynamic_modes.png` | Small-input study and frozen local spectra |

Archive keys have the form `<case>__time`, `<case>__nonlinear_delta`, `<case>__central_lti_delta` and `<case>__directional_delta`; `state_names` fixes the column order. Surface inputs can be reconstructed from the recorded trim and the case amplitude/time interval. Files with matching names are replaced on a new run; use `--output-dir` to retain another study.

The normal plot renderer is Matplotlib. Bundled charts are standard ReportLab plots rendered from the same numerical arrays; styling can differ during regeneration. The exact renderer is recorded in JSON. The data-only command and the numerical model do not depend on either plotting backend.

## Code and tests

`c172sim/linearization.py` contains the local equations and modal analysis. `run_linear_analysis.py` orchestrates trim, pulse runs, refinement and export. `c172sim/analysis_plots.py` defines the quantities shown by either renderer.

`tests/test_linearization.py` checks the drag decomposition, one-sided gap, second-order directional remainder, finite-difference convergence, unchanged physical data, moving trim/neutral coordinates, rejection of other slope corners, zero-input propagation, analytic modal frequency, unit-invariant participation and complete short-run comparison/export.

The bundled [verification record](../results/linear_analysis/verification.txt) reports 27 passing tests, including all nine new local-model tests, and eight SciPy-dependent skips out of 35 discovered tests. SciPy was unavailable in that execution environment. Run the complete suite with the declared dependencies to exercise the skipped checks; the record does not establish a full-dependency pass.
