"""Build a reduced civil C172 data set from the pinned reference XML."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from audit_reference import load_reference, FT_M, IN_M, SOURCE_COMMIT, SOURCE_URL

def skew_shift(mass, offset):
    r = np.asarray(offset, dtype=float)
    return mass * (np.dot(r, r) * np.eye(3) - np.outer(r, r))

def build(source):
    audit, load = load_reference(source)
    root = ET.parse(source).getroot()
    cg = np.array(load["derived"]["cg_structural_m"])
    structural_to_body = np.diag([-1.0, 1.0, -1.0])
    base = audit["base_aircraft"]
    # Source base tensor has zero off-diagonal terms in this pinned file.
    J = np.diag([base["inertia_components"][k]["si_value"] for k in ("ixx","iyy","izz")])
    J += skew_shift(base["mass"]["si_value"], structural_to_body @ (np.array(base["cg"]["si_value"]) - cg))
    for point in audit["source_default_point_masses"]:
        mass = load["choices"]["point_masses_kg"][point["name"]]
        J += skew_shift(mass, structural_to_body @ (np.array(point["location"]["si_value"]) - cg))
    fuel_inertia = []
    for index, tank in enumerate(root.findall("propulsion/tank")):
        mass = load["choices"]["fuel_mass_per_tank_kg"][index]
        radius = float(tank.findtext("radius")) * IN_M
        factor = float(tank.findtext("inertia_factor", "1.0"))
        # JSBSim liquid-tank convention: isotropic inertia 2/5 m R^2.
        own = 0.4 * mass * factor * radius**2
        J += own * np.eye(3)
        position = audit["source_default_fuel_tanks"][index]["location"]["si_value"]
        J += skew_shift(mass, structural_to_body @ (np.array(position) - cg))
        fuel_inertia.append({"mass_kg":mass, "radius_m":radius, "self_inertia_kg_m2":own})
    if np.min(np.linalg.eigvalsh(J)) <= 0:
        raise ValueError("Loaded inertia is not positive definite")
    functions = {f.attrib["name"].split("/")[-1]:f for f in root.findall("aerodynamics/axis/function")}
    def scalar(name):
        values = functions[name].findall("product/value")
        if len(values) != 1:
            raise ValueError(f"Unexpected scalar structure: {name}")
        return float(values[0].text)
    def table_column(name, column=1, skip_header=False):
        text = functions[name].find("product/table/tableData").text
        rows = [[float(x) for x in line.split()] for line in text.strip().splitlines() if line.strip()]
        if skip_header:
            rows = rows[1:]
        return [[row[0], row[column]] for row in rows]
    keys = ["CLDe","CLq","CDo","CDDe","CDbeta","CYda","CYdr","CYp","CYr",
            "Clp","Clda","Cldr","Cmalpha","Cmq","Cmo","Cmde","Cnp","Cnr","Cnda","Cndr"]
    tables = {
        "CL_static":table_column("CLwbh",1,True),
        "CD_alpha":table_column("CDwbh",1,True),
        "CY_beta":table_column("CYb"),
        "Cl_beta":table_column("Clb"),
        "Cl_r":table_column("Clr"),
        "Cn_beta":table_column("Cnb"),
    }
    engine_position = root.find("propulsion/engine/thruster/location")
    engine_xyz = np.array([float(engine_position.findtext(k))*IN_M for k in ("x","y","z")])
    aero_xyz = np.array(audit["aerodynamic_reference_point"]["si_value"])
    data = {
        "name":"Educational reduced C172P", "source_commit":SOURCE_COMMIT,
        "source_url":SOURCE_URL, "configuration":load["name"],
        "mass_kg":load["derived"]["total_mass_kg"],
        "inertia_body_kg_m2":J.tolist(), "cg_structural_m":cg.tolist(),
        "fuel_self_inertia":fuel_inertia,
        "inertia_reference":"JSBSim FGMassBalance.cpp and liquid branch of FGTank.cpp at the pinned commit; all non-pilot point masses zero",
        "wing_area_m2":audit["geometry"]["wingarea"]["si_value"],
        "span_m":audit["geometry"]["wingspan"]["si_value"],
        "chord_m":audit["geometry"]["chord"]["si_value"],
        "aero_reference_offset_body_m":(structural_to_body@(aero_xyz-cg)).tolist(),
        "thrust_offset_body_m":(structural_to_body@(engine_xyz-cg)).tolist(),
        "coefficients":{k:scalar(k) for k in keys},
        "tables":tables,
        "gravity_m_s2":9.80665,
        "density_kg_m3":1.1116425003060326,
        "density_description":"ISA troposphere value at 1000 m, held constant during initial comparisons.",
        "omitted_source_terms":[
            "CLadot and Cmadot acceleration-dependent contributions",
            "additional multidimensional CLalpha contribution present in c172x",
            "flaps, stall hysteresis and ground effect",
            "engine/propeller dynamics, slipstream, gyroscopic and propeller torque effects",
            "autopilot, actuators, wind, Earth curvature and rotation, fuel burn and ground contact"
        ],
        "control_interface":"Actual elevator and effective-aileron and rudder deflections in radians; prescribed axial thrust in N.",
        "study_domain":{"Va_m_s":[25,75],"alpha_deg":[-2.5,6.5],"beta_abs_deg":8,"theta_abs_deg":60},
    }
    load["status"]="mass_cg_and_inertia_defined"
    load["derived"]["inertia_loaded_kg_m2"]=J.tolist()
    load["derived"]["inertia_status"]="Assembled with parallel-axis theorem; liquid tank self inertia follows source convention."
    load["initial_flight_condition"]["atmosphere_status"]="ISA density at 1000 m held constant; no wind."
    return data, load

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    parser.add_argument("--output-dir",type=Path,default=Path("data"))
    args=parser.parse_args()
    data, load=build(args.source)
    args.output_dir.mkdir(parents=True,exist_ok=True)
    for filename, payload in (("c172_reduced_model.json",data),("load_case_v0.json",load)):
        (args.output_dir/filename).write_text(json.dumps(payload,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps({"mass_kg":data["mass_kg"],"inertia_body_kg_m2":data["inertia_body_kg_m2"],"tables":list(data["tables"])},indent=2))
if __name__=="__main__":
    main()
