# Governing equations and code correspondence

The integrated state is

\[
x=[V,\alpha,\beta,p,q,r,\phi,\theta,\psi,x_N,x_E,x_D]^T.
\]

The implementation assumes constant mass and inertia, a local flat Earth, Euler 3-2-1 attitude coordinates and still air. Units are SI, including radians for angles. Executable commands are provided in the [README](../README.md).

## Reference frames: `c172sim/frames.py`

Auxiliary body-axis velocity components are

\[
u=V\cos\alpha\cos\beta,\quad v=V\sin\beta,\quad
w=V\sin\alpha\cos\beta.
\]

Body axes point forward, right and down. The body-to-NED matrix \(C_{bn}\) transforms body components into North-East-Down components. Position rate is \(C_{bn}[u,v,w]^T\). Air-relative and ground-relative velocity coincide under the no-wind assumption.

`wind_to_body` transforms the wind-axis force vector \([-D,Y,-L]^T\) into body axes. Selected native aerodynamic moments are initially expressed in body axes.

The body angular rates p, q and r are components of the angular velocity vector. `euler_rates` maps them to Euler 3-2-1 angle rates; they are not themselves the attitude-angle derivatives.

## Translation: `c172sim/dynamics.py`

`rhs_from_loads` receives a body-frame force excluding gravity and a body-frame moment about the loaded CG:

\[
\dot{\mathbf v}_b=\frac{\mathbf F_b}{m}
+C_{bn}^{T}\begin{bmatrix}0\\0\\g\end{bmatrix}
-\boldsymbol\omega_b\times\mathbf v_b.
\]

The cross product accounts for differentiating vector components in rotating body axes. The auxiliary velocity derivatives are mapped to the integrated velocity-state derivatives:

\[
\dot V=\frac{u\dot u+v\dot v+w\dot w}{V},
\qquad
\dot\alpha=\frac{u\dot w-w\dot u}{u^2+w^2},
\]

\[
\dot\beta=\frac{\dot v-(v/V)\dot V}{V\cos\beta}.
\]

These expressions require nonzero airspeed and sideslip away from ninety degrees. The documented study domain stays clear of these singularities.

## Rotation: `c172sim/dynamics.py`

For the full constant inertia tensor \(I\) about the loaded CG,

\[
\dot{\boldsymbol\omega}
=I^{-1}\left(\mathbf M-\boldsymbol\omega\times
(I\boldsymbol\omega)\right).
\]

The implementation uses `numpy.linalg.solve` without explicitly constructing the inverse. Off-diagonal entries of the loaded tensor are retained.

## Loads: `c172sim/aircraft.py`

The module evaluates selected coefficient tables and terms, then dimensionalizes them with dynamic pressure, wing area and reference lengths. Angular-rate terms use the nondimensional factors b/(2V) or c/(2V).

Forces are transformed into body axes. The moment about the loaded CG includes the reference-point shift:

\[
\mathbf M_{CG}=\mathbf M_{ref}+
\mathbf r_{ref/CG}\times\mathbf F.
\]

This shift is applied to aerodynamic loads and prescribed thrust. Gravity acts at the CG and contributes no moment about it.

Tables use linear interpolation and hold endpoint values outside their individual intervals. This also applies to short tables such as Clr. The separate study-domain monitor constrains trajectory states; it does not establish physical validity for every table endpoint or extrapolated condition.

## Equilibrium: `c172sim/trim.py`

True airspeed, zero sideslip, zero body rates and heading are prescribed. The six unknowns are angle of attack, bank, elevator, effective aileron, rudder and thrust. The solver balances all three total force components, including gravity, and all three moments.

With zero sideslip, zero vertical ground velocity is imposed through

\[
\theta=\arctan(\cos\phi\tan\alpha).
\]

Small lateral control and bank offsets accommodate the asymmetric loading. The optimizer scales force and moment residuals; dimensional residuals and state derivatives provide additional acceptance checks.

Initial altitude sets the NED vertical coordinate. Changing that altitude alone does not update the constant density used in this release.

## Integration: `c172sim/simulation.py`

SciPy DOP853 integrates the model's `rhs` function. Each evaluation supplies twelve state derivatives; the integrator constructs their time history.

The elevator pulse is represented by constant-control intervals. The final state of each interval initializes the next, placing input discontinuities explicitly at segment boundaries.

## Verification evidence and limits

Tests compare analytic solutions and known physical invariants: gravity-only motion, energy and inertial angular momentum conservation in torque-free rotation, reference-frame geometry and numerical convergence. Equilibrium persistence adds a consistency check.

Aerodynamic prediction quality also depends on the selected data and simplifications in `aircraft.py`. There is no quantitative comparison with a JSBSim execution or measured flight data in this release.

## References

- [MathWorks: 6DOF with Euler angles](https://www.mathworks.com/help/aeroblks/6dofeulerangles.html).
- [JSBSim: body, wind and NED reference frames](https://jsbsim-team.github.io/jsbsim-reference-manual/user/concepts/frames-of-reference/).
- [SciPy: solve_ivp](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html).
