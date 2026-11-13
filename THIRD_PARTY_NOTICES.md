# Third-party source information

Reference: JSBSim-Team/jsbsim, aircraft/c172x/c172x.xml, authored by Tony Peden, at commit c455897049e044860b3c282e955daca1d0a785fe.

https://github.com/JSBSim-Team/jsbsim/blob/c455897049e044860b3c282e955daca1d0a785fe/aircraft/c172x/c172x.xml

The aircraft XML declares GPL licensing and contains additional source notices. Consult and retain those notices if redistributing upstream code/files. This package does not bundle that XML. Its JSON records selected source data, conversions and provenance; the rigid-body flight model is an educational implementation.

Version 0.5.0 adds a stationary adaptation of the JSBSim propulsion equations in `c172sim/propulsion.py`. This adapted module is distributed under LGPL-2.1-or-later, with modification and copyright notices in its header. The upstream license is included in [LICENSES/JSBSim-LGPL.txt](LICENSES/JSBSim-LGPL.txt). It is a reduced stationary implementation, not the complete JSBSim propulsion system.

Propulsion sources at the same pinned commit:

- `engine/eng_io320.xml`: Ron Jensen; motor configuration.
- `engine/prop_75in2f.xml`: fixed-pitch propeller configuration and active tables.
- `src/models/propulsion/FGPiston.cpp`: copyright (C) 2000 Jon S. Berndt; engine modelling by Dave Luff and Ronald Jensen.
- `src/models/propulsion/FGPropeller.cpp`: copyright (C) 2000 Jon S. Berndt.

Exact URLs, source Git blob identities and SHA-256 hashes are recorded in `data/c172_propulsion.json`. Constructor defaults and the default mixture-efficiency table are identified separately from XML parameters. No manufacturer-origin fuel-flow map or independent C172P manual validation is claimed. `tools/build_propulsion_data.py` reproduces the data extraction from identity-checked source files.

Upstream lists a 1982 Cessna 172P information manual and FAA Type Certificate Data Sheet 3A12. Those original documents were not independently audited here.

Mass-property conventions were checked against:
- https://github.com/JSBSim-Team/jsbsim/blob/c455897049e044860b3c282e955daca1d0a785fe/src/models/FGMassBalance.cpp
- https://github.com/JSBSim-Team/jsbsim/blob/c455897049e044860b3c282e955daca1d0a785fe/src/models/propulsion/FGTank.cpp

Aerodynamic reference conventions:
- https://github.com/JSBSim-Team/jsbsim/blob/c455897049e044860b3c282e955daca1d0a785fe/src/models/FGAerodynamics.cpp
- https://jsbsim-team.github.io/jsbsim-reference-manual/user/concepts/frames-of-reference/

SciPy provides numerical integration and optimization; NumPy provides arrays and linear algebra; Matplotlib provides plotting. No JSBSim runtime is imported. There is no claim of manufacturer endorsement or measured flight-data validation.
