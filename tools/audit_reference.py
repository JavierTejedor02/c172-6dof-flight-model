"""Audit a pinned civil-aircraft reference file; standard library only.

This tool inventories source data and converts units. It is not a flight
simulator and does not evaluate aerodynamic or propulsion functions.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

SOURCE_REPOSITORY = "JSBSim-Team/jsbsim"
SOURCE_PATH = "aircraft/c172x/c172x.xml"
SOURCE_COMMIT = "c455897049e044860b3c282e955daca1d0a785fe"
EXPECTED_GIT_BLOB = "8f121c16eb81765b5ab672f80a4404e042fd179d"
SOURCE_URL = f"https://github.com/{SOURCE_REPOSITORY}/blob/{SOURCE_COMMIT}/{SOURCE_PATH}"
FT_M = 0.3048
IN_M = 0.0254
LB_KG = 0.45359237
G0 = 9.80665
SLUG_KG = LB_KG * G0 / FT_M
CONVERSIONS = {
    "FT": (FT_M, "m"),
    "IN": (IN_M, "m"),
    "FT2": (FT_M**2, "m^2"),
    "LBS": (LB_KG, "kg"),
    "SLUG*FT2": (SLUG_KG * FT_M**2, "kg*m^2"),
}

def quantity(parent: ET.Element, tag: str) -> dict:
    element = parent.find(tag)
    if element is None or element.text is None:
        raise ValueError(f"Missing quantity: {tag}")
    value = float(element.text)
    unit = element.attrib["unit"]
    if unit not in CONVERSIONS:
        raise ValueError(f"Unsupported unit: {unit}")
    factor, si_unit = CONVERSIONS[unit]
    if not math.isfinite(value):
        raise ValueError(f"Non-finite value: {tag}")
    return {"source_value": value, "source_unit": unit,
            "si_value": value * factor, "si_unit": si_unit}

def location(parent: ET.Element, selector: str) -> dict:
    element = parent.find(selector)
    if element is None:
        raise ValueError(f"Missing location: {selector}")
    values = [float(element.findtext(axis)) for axis in ("x", "y", "z")]
    factor, unit = CONVERSIONS[element.attrib["unit"]]
    if unit != "m" or not all(math.isfinite(v) for v in values):
        raise ValueError("Invalid location")
    return {"source_value": values, "source_unit": element.attrib["unit"],
            "si_value": [v * factor for v in values], "si_unit": "m",
            "frame": "upstream structural frame: x aft, y right, z up"}

def load_reference(source: Path) -> tuple[dict, dict]:
    raw = source.read_bytes()
    blob = hashlib.sha1(b"blob " + str(len(raw)).encode() + b"\0" + raw).hexdigest()
    if blob != EXPECTED_GIT_BLOB:
        raise ValueError(
            f"Source differs from pinned Git blob: {blob}; expected {EXPECTED_GIT_BLOB}. "
            "Download the exact pinned file without newline conversion."
        )
    root = ET.fromstring(raw)
    metrics = root.find("metrics")
    balance = root.find("mass_balance")
    propulsion = root.find("propulsion")
    aero = root.find("aerodynamics")
    if any(x is None for x in (metrics, balance, propulsion, aero)):
        raise ValueError("Incomplete source aircraft")

    geometry = {}
    for key in ("wingarea", "wingspan", "chord", "htailarea", "htailarm",
                "vtailarea", "vtailarm"):
        geometry[key] = {**quantity(metrics, key), "source_xpath": f"./metrics/{key}"}
    empty = {
        "mass": {**quantity(balance, "emptywt"), "source_xpath": "./mass_balance/emptywt"},
        "cg": location(balance, "./location[@name='CG']"),
        "inertia_components": {
            key: {**quantity(balance, key), "source_xpath": f"./mass_balance/{key}"}
            for key in ("ixx", "iyy", "izz", "ixz")
        },
        "interpretation": "Base-aircraft properties. Do not use as loaded-aircraft inertia.",
    }
    inertias = [empty["inertia_components"][k]["si_value"] for k in ("ixx","iyy","izz")]
    if not all(x > 0 for x in inertias):
        raise ValueError("Nonpositive diagonal inertia")
    if any(inertias[i] > sum(inertias[j] for j in range(3) if j != i) for i in range(3)):
        raise ValueError("Principal-inertia triangle inequality fails")

    point_masses = []
    for item in balance.findall("pointmass"):
        point_masses.append({
            "name": item.attrib["name"],
            "default_mass": quantity(item, "weight"),
            "location": location(item, "./location"),
        })
    tanks = []
    for index, item in enumerate(propulsion.findall("tank")):
        tanks.append({
            "index": index, "type": item.attrib["type"],
            "capacity": quantity(item, "capacity"),
            "default_contents": quantity(item, "contents"),
            "location": location(item, "./location"),
        })
    if len(tanks) != 2 or any(t["type"] != "FUEL" for t in tanks):
        raise ValueError("Unexpected tank configuration")

    inventory = {}
    for axis in aero.findall("axis"):
        inventory[axis.attrib["name"]] = [
            function.attrib.get("name") for function in axis.findall("function")
        ]
    audit = {
        "schema_version": 1,
        "purpose": "Civil Cessna 172P reference-data inventory; not a flight simulator",
        "source": {
            "repository": SOURCE_REPOSITORY, "path": SOURCE_PATH,
            "commit": SOURCE_COMMIT, "git_blob_sha1": blob,
            "sha256": hashlib.sha256(raw).hexdigest(), "byte_count": len(raw),
            "url": SOURCE_URL, "retrieved_date": "2026-11-04",
            "author": root.findtext("fileheader/author"),
            "description": root.findtext("fileheader/description"),
            "license_declaration": root.find("fileheader/license").attrib,
            "limitations": "Upstream educational model; mixes public data and estimates; limited validation.",
            "references_listed_by_upstream": [
                item.attrib for item in root.findall("fileheader/reference")
            ],
        },
        "unit_conversion_factors": {
            "ft_to_m": FT_M, "in_to_m": IN_M, "lb_to_kg": LB_KG,
            "slug_to_kg": SLUG_KG, "slug_ft2_to_kg_m2": SLUG_KG * FT_M**2,
        },
        "geometry": geometry,
        "aerodynamic_reference_point": location(metrics, "./location[@name='AERORP']"),
        "base_aircraft": empty,
        "source_default_point_masses": point_masses,
        "source_default_fuel_tanks": tanks,
        "aerodynamic_inventory": inventory,
        "propulsion_file_references": {
            "engine": propulsion.find("engine").attrib.get("file"),
            "propeller": propulsion.find("engine/thruster").attrib.get("file"),
            "dependency_status": "Component names only; propulsion parameters are extracted separately by tools/build_propulsion_data.py.",
        },
        "warnings": [
            "Source configuration includes extra point masses; defaults are not the selected load case.",
            "Locations are in structural axes, not body axes.",
            "Base-aircraft inertia is not the inertia of the loaded aircraft.",
            "Source namespace 'aero/coefficient' does not guarantee a dimensionless output; inspect each function.",
            "Geometric wingarea/wingspan and source reference chord are distinct source quantities.",
            "Agreement with another simulator is not flight-test validation.",
        ],
    }

    # Explicit project choices, distinct from source defaults.
    selected_points = {item["name"]: 0.0 for item in point_masses}
    if "PILOT" not in selected_points:
        raise ValueError("Missing pilot entry")
    selected_points["PILOT"] = 190.0 * LB_KG
    fuel_per_tank_kg = 65.0 * LB_KG
    if any(fuel_per_tank_kg > tank["capacity"]["si_value"] for tank in tanks):
        raise ValueError("Fuel selection exceeds a tank capacity")
    masses = [empty["mass"]["si_value"]]
    positions = [empty["cg"]["si_value"]]
    for item in point_masses:
        masses.append(selected_points[item["name"]])
        positions.append(item["location"]["si_value"])
    for tank in tanks:
        masses.append(fuel_per_tank_kg)
        positions.append(tank["location"]["si_value"])
    total_mass = math.fsum(masses)
    cg = [
        math.fsum(mass * pos[i] for mass, pos in zip(masses, positions)) / total_mass
        for i in range(3)
    ]
    expected_mass = (1454.0 + 190.0 + 2 * 65.0) * LB_KG
    if not math.isclose(total_mass, expected_mass, rel_tol=1e-12):
        raise ValueError("Load-case mass balance failed")

    load_case = {
        "name": "pilot_half_fuel_v0",
        "status": "mass_and_cg_defined; loaded_inertia_pending",
        "reference_commit": SOURCE_COMMIT,
        "choices": {
            "description": "One pilot, half fuel in each tank, other point masses set to zero.",
            "pilot_mass_kg": selected_points["PILOT"],
            "point_masses_kg": selected_points,
            "fuel_mass_per_tank_kg": [fuel_per_tank_kg] * 2,
            "fuel_capacity_fraction": 0.5,
            "fuel_burn": "Frozen for initial short-duration comparisons.",
            "flap_setting_rad": 0.0,
        },
        "derived": {
            "total_mass_kg": total_mass,
            "cg_structural_m": cg,
            "cg_frame": "upstream structural frame: x aft, y right, z up",
            "inertia_loaded_kg_m2": None,
            "inertia_status": "Pending assembly consistent with the reference model; do not substitute base inertia.",
        },
        "initial_flight_condition": {
            "status": "Target only; trim has not been solved.",
            "true_airspeed_target_m_s": 50.0,
            "altitude_target_m": 1000.0,
            "environment": "local flat Earth, no wind",
            "atmosphere_status": "Density/model to be fixed before computing trim.",
            "alpha_rad": None, "theta_rad": None,
            "controls": None,
        },
    }
    return audit, load_case

def write_json(destination: Path, content: dict) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(content, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=Path("data"))
    args = parser.parse_args()
    audit, load_case = load_reference(args.source)
    write_json(args.output_dir / "c172_reference.json", audit)
    write_json(args.output_dir / "load_case_v0.json", load_case)
    print(json.dumps({
        "source_blob_verified": audit["source"]["git_blob_sha1"],
        "mass_kg": load_case["derived"]["total_mass_kg"],
        "cg_structural_m": load_case["derived"]["cg_structural_m"],
        "aero_axis_count": len(audit["aerodynamic_inventory"]),
        "loaded_inertia": "pending",
        "written": [str(args.output_dir / name)
                    for name in ("c172_reference.json", "load_case_v0.json")],
    }, indent=2))

if __name__ == "__main__":
    main()
