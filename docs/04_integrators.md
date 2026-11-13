# Numerical integrators and comparison experiment

## Purpose

Euler, classical RK4 and DOP853 solve the same twelve-state C172 model with the same loaded properties, atmosphere, initial trim and elevator pulse. Changing the integrator does not change the force/moment equations. This experiment isolates numerical accuracy and cost.

The integrator comparison was introduced in version 0.2.0. Earlier bundled trajectory files and `results/test_run.txt` remain reference records from version 0.1.0; they are not measurements of that experiment.

## From derivatives to a trajectory

`c172sim/dynamics.py` defines the first-order system:

$$
\dot{\mathbf{x}}=f(t,\mathbf{x},\mathbf{u}).
$$

The integrator takes the current state, evaluates this function and advances time. Each evaluation recomputes forces and moments for that evaluation's state. All twelve components advance together, including position, attitude and body angular rates.

The formulas are explicit in [integrators.py](../c172sim/integrators.py). The control-segment loop and domain checks are in [simulation.py](../c172sim/simulation.py).

## Explicit Euler

$$
\mathbf{x}_{n+1}=\mathbf{x}_n+h f(t_n,\mathbf{x}_n,\mathbf{u}_n).
$$

The code consists of one derivative evaluation and one update:

```python
derivative = fun(time, state)
next_state = state + step_size * derivative
```

Euler assumes the initial slope represents the whole step. It is first order: on a sufficiently smooth, stable problem, halving the step approximately halves the accumulated error over a fixed duration. Large steps can introduce excessive numerical damping, artificial growth or instability.

## Classical RK4

$$
\begin{aligned}
k_1 &= f(t_n,x_n),\\
k_2 &= f(t_n+h/2,x_n+h k_1/2),\\
k_3 &= f(t_n+h/2,x_n+h k_2/2),\\
k_4 &= f(t_n+h,x_n+h k_3),\\
x_{n+1} &= x_n+\frac{h}{6}(k_1+2k_2+2k_3+k_4).
\end{aligned}
$$

Controls are constant within each segment and are omitted from the formula for brevity. RK4 uses four derivative evaluations per step. On sufficiently smooth problems, its fourth-order global error decreases by approximately sixteen when the step is halved. The aircraft's piecewise-linear tables can reduce this ideal rate; smooth analytic equations are used to test the formulas' formal orders.

## Adaptive DOP853

SciPy `solve_ivp(..., method="DOP853")` supplies an explicit Runge-Kutta method of order eight with adaptive error control. It remains the default. `rtol` and `atol` govern local error estimates; `max_step` is an upper bound, not a forced step size. See the [SciPy documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html).

In `simulate_segments`, the parameters have distinct meanings:

| Parameter | Euler / RK4 | DOP853 |
|---|---|---|
| `step_size` | Nominal integration step; save every node | Unused |
| `dt` | Unused | Regular output interval |
| `max_step` | Unused | Maximum internal adaptive step |
| `rtol`, `atol` | Unused | Local error-control tolerances |
| `output_times` | Not supported; use returned nodes | Optional exact requested observation times |

The CLI option `--step-size` maps to `step_size` for the fixed-step methods and to `max_step` for DOP853. The comparison uses the same mapping for each value in `--steps`.

## Controls and domain boundaries

Every method splits integration at 2 and 3 seconds, the pulse boundaries. No step crosses an input discontinuity. A fixed step is shortened when necessary to land exactly on a segment's end, and the end is saved only once. Controls stored at a shared boundary describe the segment that just ended; subsequent integration uses the next segment's controls.

DOP853 locates a domain-boundary event. Euler and RK4 check saved nodes, and RK4 also checks intermediate stages; they do not locate an exact crossing time. Failed runs are reported with no error or timing score and are omitted from plots. A failure alone does not distinguish numerical instability from a genuinely out-of-domain trajectory.

## Execute from the repository root

Complete installation using the [README](../README.md), then run:

```powershell
.\.venv\Scripts\python.exe compare_longitudinal_integrators.py
```

Defaults: 30 seconds, -0.5 degree elevator increment from 2 to 3 seconds, steps of 0.2, 0.1, 0.05 and 0.025 seconds, and three complete integrations per timing measurement. The script can also be launched without arguments using an editor's Python Run action.

For a shorter experiment:

```powershell
.\.venv\Scripts\python.exe compare_longitudinal_integrators.py --duration 10 --steps 0.1 0.05 0.025 --repeats 1
```

For standalone Euler or RK4 trajectories:

```powershell
.\.venv\Scripts\python.exe run_longitudinal_demo.py --method EULER --step-size 0.025 --output-dir results/euler_demo
.\.venv\Scripts\python.exe run_longitudinal_demo.py --method RK4 --step-size 0.05 --output-dir results/rk4_demo
```

## Numerical reference and metrics

Each candidate starts from the same computed trim. A tight DOP853 reference uses `rtol=1e-11`, `atol=1e-13`, `max_step=0.025`. A second run uses `rtol=1e-13`, `atol=1e-15`, `max_step=0.0125`. The reference is accepted only if the maximum scaled difference is below `1e-5`. Settings and measured differences are recorded.

The reference is sampled directly using DOP853's continuous interpolant at the union of candidate observation times. Errors use identical timestamps within each candidate/reference pair without linearly interpolating low-order trajectories. Maxima and RMS are over saved nodes, not guaranteed continuous-time error bounds. Different steps produce different sample grids.

Metrics include maximum airspeed, pitch, pitch-rate, altitude and three-dimensional position errors, plus dimensional errors for all twelve states. Combined dimensionless errors divide each component by a declared scale: 1 m/s for airspeed, 1 degree for angles, 1 degree/s for angular rates and 1 metre for positions. These are comparison scales, not operating limits or accuracy requirements.

Cost is measured with derivative evaluations (`nfev`) and median elapsed integration time. Timing includes load evaluation, domain checks and output sampling; it excludes trim, reference construction, file writing and plotting. A warm-up precedes measurement. Hardware, background processes and method order affect timing; `nfev` is complementary and does not imply every evaluation has identical cost.

The reference is numerical, not exact. Errors near the reference-refinement difference cannot be ranked reliably. This experiment does not validate the aerodynamics against flight data.

## Outputs

Default directory: `results/longitudinal_integrator_comparison/`. `--output-dir` selects another location. Generated files with the same names are replaced; consult the current report's completed-run list when inspecting CSVs from a reused directory.

| File | Contents |
|---|---|
| `comparison.csv` | One row per method/step: status, errors, RHS calls and elapsed time |
| `comparison.json` | Settings, environment, reference check, per-state errors and failure messages |
| `reference.csv` | Tight-reference trajectory at comparison timestamps |
| `<method>_h<step>.csv` | Completed candidate trajectory and controls |
| `trajectories.png` | Speed, pitch and altitude histories grouped by integrator |
| `accuracy_and_cost.png` | Error versus step, altitude error, error versus RHS calls and timing |

Numerical values are produced by running the experiment; no unmeasured comparison results are bundled. Equilibrium drift alone is insufficient to compare integrators, which is why the experiment uses a perturbed trajectory.

## Verification

`tests/test_integrator_steps.py` checks analytic exponential convergence, coupled constant-acceleration motion, a time-dependent derivative and a harmonic oscillator. These formula tests need NumPy but not SciPy. `tests/test_fixed_simulation.py` uses the recorded equilibrium as its initial-state fixture and checks flight convergence, shortened boundary steps, control switching, evaluation counts, step/output separation, invalid input and domain exit. Its DOP853-reference check requires SciPy. The complete suite requires `requirements.txt` dependencies.

`tests/test_longitudinal_integrator_metrics.py` verifies physical error-unit scaling and exact reference timestamp matching without requiring SciPy or Matplotlib.

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```
