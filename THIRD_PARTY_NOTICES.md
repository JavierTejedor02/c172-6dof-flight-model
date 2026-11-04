# Third-party source information

Reference: JSBSim-Team/jsbsim, aircraft/c172x/c172x.xml, authored by Tony Peden, at commit c455897049e044860b3c282e955daca1d0a785fe.

https://github.com/JSBSim-Team/jsbsim/blob/c455897049e044860b3c282e955daca1d0a785fe/aircraft/c172x/c172x.xml

The XML declares GPL licensing and contains additional source notices. Consult and retain those notices if redistributing upstream code/files. This package does not bundle the XML or engine/propeller implementation. Its JSON records selected source data, conversions and provenance; the Python flight model is a separate educational implementation.

Upstream lists a 1982 Cessna 172P information manual and FAA Type Certificate Data Sheet 3A12. Those original documents were not independently audited here.

Mass-property conventions were checked against:
- https://github.com/JSBSim-Team/jsbsim/blob/c455897049e044860b3c282e955daca1d0a785fe/src/models/FGMassBalance.cpp
- https://github.com/JSBSim-Team/jsbsim/blob/c455897049e044860b3c282e955daca1d0a785fe/src/models/propulsion/FGTank.cpp

Aerodynamic reference conventions:
- https://github.com/JSBSim-Team/jsbsim/blob/c455897049e044860b3c282e955daca1d0a785fe/src/models/FGAerodynamics.cpp
- https://jsbsim-team.github.io/jsbsim-reference-manual/user/concepts/frames-of-reference/

SciPy provides numerical integration and optimization; NumPy provides arrays and linear algebra; Matplotlib provides plotting. No JSBSim runtime is imported. There is no claim of manufacturer endorsement or measured flight-data validation.
