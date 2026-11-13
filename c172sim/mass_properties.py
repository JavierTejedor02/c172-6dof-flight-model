"""Reassemble the selected civil load case for symmetric remaining tank masses."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
from .aircraft import Aircraft

DATA_DIR = Path(__file__).resolve().parents[1] / "data"
STRUCTURAL_TO_BODY = np.diag([-1.0, 1.0, -1.0])


def parallel_axis(mass, offset):
    return mass * (np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset))


class FuelLoadCase:
    """Independent Aircraft snapshots; the original short-run model stays fixed."""
    def __init__(self):
        self.template = Aircraft()
        self.audit = json.loads((DATA_DIR / "c172_reference.json").read_text(encoding="utf-8"))
        self.load = json.loads((DATA_DIR / "load_case_v0.json").read_text(encoding="utf-8"))
        if self.audit["source"]["commit"] != self.template.data["source_commit"]:
            raise ValueError("Mass-property source mismatch")
        self.capacity_kg = sum(t["capacity"]["si_value"] for t in self.audit["source_default_fuel_tanks"])
        self.initial_fuel_kg = sum(self.load["choices"]["fuel_mass_per_tank_kg"])

    def aircraft(self, fuel_kg):
        if not np.isfinite(fuel_kg) or not 0 <= fuel_kg <= self.capacity_kg:
            raise ValueError("Remaining fuel must lie within the selected tank capacity")
        base = self.audit["base_aircraft"]
        masses = [base["mass"]["si_value"]]
        positions = [base["cg"]["si_value"]]
        for point in self.audit["source_default_point_masses"]:
            masses.append(self.load["choices"]["point_masses_kg"][point["name"]])
            positions.append(point["location"]["si_value"])
        for tank in self.audit["source_default_fuel_tanks"]:
            masses.append(fuel_kg / 2)
            positions.append(tank["location"]["si_value"])
        masses, positions = np.asarray(masses), np.asarray(positions)
        cg = np.sum(masses[:, None] * positions, axis=0) / np.sum(masses)
        inertia = np.diag([base["inertia_components"][key]["si_value"] for key in ("ixx", "iyy", "izz")])
        if base["inertia_components"]["ixz"]["si_value"] != 0:
            raise ValueError("This assembler requires the pinned diagonal empty-aircraft tensor")
        for mass, position in zip(masses, positions):
            inertia += parallel_axis(mass, STRUCTURAL_TO_BODY @ (position - cg))
        for tank in self.template.data["fuel_self_inertia"]:
            # Same fixed-radius isotropic liquid self-inertia convention as the source.
            inertia += np.eye(3) * (fuel_kg / 2) * tank["self_inertia_kg_m2"] / tank["mass_kg"]
        if np.min(np.linalg.eigvalsh(inertia)) <= 0:
            raise ValueError("Invalid assembled inertia")
        result = deepcopy(self.template)
        old_cg = np.asarray(result.data["cg_structural_m"])
        shift = STRUCTURAL_TO_BODY @ (old_cg - cg)
        result.mass, result.inertia = float(np.sum(masses)), inertia
        result.aero_offset += shift
        result.thrust_offset += shift
        result.data.update(mass_kg=result.mass, inertia_body_kg_m2=inertia.tolist(),
                           cg_structural_m=cg.tolist(), remaining_fuel_kg=float(fuel_kg),
                           aero_reference_offset_body_m=result.aero_offset.tolist(),
                           thrust_offset_body_m=result.thrust_offset.tolist())
        for tank in result.data["fuel_self_inertia"]:
            tank["self_inertia_kg_m2"] *= (fuel_kg / 2) / tank["mass_kg"]
            tank["mass_kg"] = fuel_kg / 2
        return result
