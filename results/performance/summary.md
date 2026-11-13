# Sampled quasi-steady cruise comparison

Constant 1000 m atmosphere, no wind; fuel/air ratio fixed; equal tank depletion.
All strategies burn the same fuel mass. Climb, descent and speed-transition costs are omitted.

| Sampled strategy | Duration [h] | Distance [km] | Initial speed [m/s] | Final speed [m/s] | Feasible-edge selection |
|---|---:|---:|---:|---:|---|
| maximum_endurance | 4.4358 | 518.218 | 33.00 | 32.00 | False |
| maximum_range | 4.3145 | 621.484 | 40.50 | 39.50 | False |
| fixed_speed | 3.0581 | 550.450 | 50.00 | 50.00 | False |

The word maximum refers only to accepted sampled operating points. An edge selection is domain-constrained; a lower-speed optimum outside the model domain cannot be assessed.
Numerical refinement and source-model consistency are not flight-test validation.
