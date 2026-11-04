# Reference-data audit and reduced-model decisions

Source: [pinned c172x.xml](https://github.com/JSBSim-Team/jsbsim/blob/c455897049e044860b3c282e955daca1d0a785fe/aircraft/c172x/c172x.xml).

The original audit checks the exact Git blob and preserves source and SI values in `data/c172_reference.json`. This release adds `tools/build_model_data.py` to assemble the selected loaded properties and extract a deliberately limited aerodynamic data set.

## Geometry and load

Wing area: 16.16512896 m^2; span: 10.9728 m; source reference chord: 1.49352 m. The source chord is retained rather than substituted with area/span.

Base mass is 659.52330598 kg. The selected loading adds a 190 lb pilot and 65 lb of fuel in each of two tanks. Every other upstream point mass is explicitly zeroed. Total mass is 804.67286438 kg.

The CG remains in the source structural frame in the audit files. It is later used to compute body-frame offsets. The pilot is left of the symmetry plane; the lateral CG coordinate is retained.

## Loaded inertia

The base diagonal tensor is moved from the base CG to the loaded CG. Pilot and fuel mass shifts are added using the full parallel-axis tensor, preserving products of inertia. Pilot intrinsic inertia is zero, consistent with its source point-mass representation.

Each liquid tank contributes isotropic self inertia 2/5 m R^2 times the source inertia factor, with R=29.4 inches and inertia factor 1. This follows the liquid branch of [FGTank.cpp](https://github.com/JSBSim-Team/jsbsim/blob/c455897049e044860b3c282e955daca1d0a785fe/src/models/propulsion/FGTank.cpp). Source CG and inertia assembly conventions were checked in [FGMassBalance.cpp](https://github.com/JSBSim-Team/jsbsim/blob/c455897049e044860b3c282e955daca1d0a785fe/src/models/FGMassBalance.cpp).

Loaded body-frame tensor, kg m^2:

```text
[[1813.99081880,    3.87455561,   -9.57738587],
 [   3.87455561, 1869.91620833,    9.99442278],
 [  -9.57738587,    9.99442278, 3170.24502885]]
```

This is a tensor; off-diagonal entries must not be confused with differently signed scalar product-of-inertia conventions.

## Selected aerodynamic content

| Runtime data | Source function(s) | Decision |
|---|---|---|
| Static lift versus alpha | CLwbh | First table column: un-stalled branch |
| Drag versus alpha | CDwbh and CDo | Zero-flap column plus baseline |
| Elevator contributions | CLDe, CDDe, Cmde | Retain |
| Pitch stability/damping | Cmo, Cmalpha, Cmq, CLq | Retain |
| Lateral stability | CYb, Clb, Cnb | Retain tables |
| Lateral control/damping | CYda, CYdr, CYp, CYr, Clp, Clr, Clda, Cldr, Cnp, Cnr, Cnda, Cndr | Retain selected source terms |

Source quantities named under `aero/coefficient` can include dimensional scaling. The builder extracts the nondimensional constants or tables specifically used by the implemented equations; it does not treat every XML function output as a coefficient.

## Deliberate differences from full c172x

The additional multidimensional CLalpha contribution is omitted; it is not accidentally counted twice or silently assumed absent upstream. CLadot and Cmadot are omitted, so the reduced dynamic response will differ. Flaps are fixed at zero; ground-effect factors are fixed to unity; stall/hysteresis is outside scope.

No engine/propeller, autopilot, actuator, sensor or ground-contact functions are imported. The prescribed thrust input replaces the engine/propeller calculation in this release. Source force and moment reference conventions were checked against [FGAerodynamics.cpp](https://github.com/JSBSim-Team/jsbsim/blob/c455897049e044860b3c282e955daca1d0a785fe/src/models/FGAerodynamics.cpp).

Because of these differences, agreement with full JSBSim should not be expected without first aligning the models. No measured flight-data validation is claimed.

## Rebuild

Run both `audit_reference.py` and `build_model_data.py` on the pinned XML. The final runtime input is `data/c172_reduced_model.json`. The packaged runtime data avoid a network requirement for executing the example.
