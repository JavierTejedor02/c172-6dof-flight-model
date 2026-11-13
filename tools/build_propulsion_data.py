"""Extract propulsion data from four identity-checked, pinned JSBSim files."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

COMMIT = "c455897049e044860b3c282e955daca1d0a785fe"
FILES = {
    "eng_io320.xml": ("engine/eng_io320.xml", "bbfb976f0a592c8b62baa39785f174c7e70c17f3"),
    "prop_75in2f.xml": ("engine/prop_75in2f.xml", "82925a39e8c4804df9922406a5bb9d1f81a69c4c"),
    "FGPiston.cpp": ("src/models/propulsion/FGPiston.cpp", "f8144108dd462b331ea30864dd6e260ced6f515b"),
    "FGPropeller.cpp": ("src/models/propulsion/FGPropeller.cpp", "3448416a2ff6e50d45ded0502181e88114a53c95"),
}


def build(directory):
    contents, sources = {}, []
    for name, (path, expected) in FILES.items():
        raw = (Path(directory) / name).read_bytes()
        blob = hashlib.sha1(b"blob " + str(len(raw)).encode() + b"\0" + raw).hexdigest()
        if blob != expected:
            raise ValueError(f"Pinned source identity failed: {name}; preserve original bytes")
        contents[name] = raw.decode("utf-8")
        sources.append({"path": path, "git_blob_sha1": blob,
                        "sha256": hashlib.sha256(raw).hexdigest(),
                        "url": f"https://github.com/JSBSim-Team/jsbsim/blob/{COMMIT}/{path}"})
    engine = ET.fromstring(contents["eng_io320.xml"])
    prop = ET.fromstring(contents["prop_75in2f.xml"])
    names = ("minmp", "maxmp", "displacement", "maxhp", "bsfc", "cycles",
             "idlerpm", "maxrpm", "minthrottle", "maxthrottle")
    parameters = {name: float(engine.findtext(name)) for name in names}
    # These values are defaults in the checked constructor, not extra XML data.
    defaults = {"volumetric_efficiency": 0.85, "Stroke": 4.375, "CompressionRatio": 8.5,
                "PeakMeanPistonSpeed_fps": 100.0, "FMEPDynamic": 18400.0,
                "FMEPStatic": 46500.0, "StaticFriction_HP": 1.5}
    cpp = contents["FGPiston.cpp"]
    for name, value in defaults.items():
        match = re.search(r"\b" + name + r"\s*=\s*([\d.]+)\s*;", cpp)
        if match is None or float(match[1]) != value:
            raise ValueError(f"Unexpected engine default: {name}")
    mixture = [[float(a), float(b)] for a, b in re.findall(
        r"\*Mixture_Efficiency_Correlation\s*<<\s*([\d.]+)\s*<<\s*([\d.]+)", cpp)]
    if len(mixture) != 15:
        raise ValueError("Unexpected default mixture table")
    tables = {table.attrib["name"]: [[float(v) for v in line.split()]
              for line in table.findtext("tableData").strip().splitlines()]
              for table in prop.findall("table")}
    return {"source_commit": COMMIT, "sources": sources,
            "attribution": "Engine XML: Ron Jensen. FGPiston: Jon S. Berndt, Dave Luff, Ronald Jensen. FGPropeller: Jon S. Berndt. Adapted steady equations: LGPL-2.1-or-later; see LICENSES/JSBSim-LGPL.txt.",
            "engine_xml": parameters, "engine_constructor_defaults": defaults,
            "mixture_efficiency": mixture, "diameter_m": float(prop.findtext("diameter")) * 0.0254,
            "propeller_tables": tables,
            "scope": "Stationary naturally aspirated engine, both magnetos, fixed-pitch propeller; thermal, startup, lag and rotational transients omitted. Tables retain active XML entries only."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("data/c172_propulsion.json"))
    args = parser.parse_args()
    payload = build(args.source_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(args.output)
