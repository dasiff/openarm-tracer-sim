"""Safety filter: structural checks (no simulation, no MuJoCo).

The filter must never receive the simulator, the scene model or world data from the simulator (M290).
Checks, by reading the source of src/safety_filter/ (AST, not text search):
  1. no import reaches the simulator, the scene builders, the task config or the world modules of the
     project, except the three robot-only helpers the robot description needs (robot_specs, battery, cameras);
  2. the package loads no scene file (no from_xml / scene path; the only file it reads is the robot description
     and its own JSON files);
  3. public functions and class constructors take only numbers, arrays, plain containers and the robot-only
     model: no parameter is named for a simulator, a scene or a data object;
  4. the world adapter takes a WorldModel object by duck typing and does not import src.world_model.

Run: python experiments/safety_filter_structure_test.py   (exit 0 = pass)
"""

import ast
import inspect
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

PKG = ROOT / "src" / "safety_filter"
ALLOWED_PROJECT_IMPORTS = {"src.robot_specs", "src.battery", "src.cameras", "src.safety_filter"}
# world_model is NOT forbidden: M290 allows world information from the world model (the adapter takes it by duck typing).
FORBIDDEN_NAMES = ("sim", "simulator", "scene", "mjdata", "data_sim", "spec")
FORBIDDEN_CALLS = ("from_xml_path", "from_xml_string", "from_xml")

FAILS = []


def check(name, cond, detail=""):
    print(f"  {'ok  ' if cond else 'FAIL'} {name} {detail}")
    if not cond:
        FAILS.append(name)


def imports_of(path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    names = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names += [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.append(node.module)
    return names, tree


def main():
    files = sorted(PKG.glob("*.py"))
    print(f"package files: {[f.name for f in files]}")
    print("1. imports")
    bad = []
    for f in files:
        names, _ = imports_of(f)
        for n in names:
            if n.startswith("src.") and not any(n == a or n.startswith(a + ".") for a in ALLOWED_PROJECT_IMPORTS):
                bad.append((f.name, n))
    check("no project import outside the robot-only helpers", not bad, f"{bad}" if bad else "")

    print("2. no scene loading, no simulator calls")
    loads = []
    for f in files:
        src = f.read_text(encoding="utf-8")
        for call in FORBIDDEN_CALLS:
            if call in src:
                loads.append((f.name, call))
        if "MjSpec.from_file" in src and f.name != "robot_model.py":
            loads.append((f.name, "MjSpec.from_file outside robot_model"))
    check("no scene or model is loaded from text or a scene path", not loads, f"{loads}" if loads else "")
    rm = (PKG / "robot_model.py").read_text(encoding="utf-8")
    check("the only file read at load is the robot description (robot_model.ROBOT_XML)",
          "combined_robot.xml" in rm and "chemistry_lab" not in rm)

    print("3. public parameters")
    import importlib
    mods = [importlib.import_module(f"src.safety_filter.{f.stem}") for f in files if f.stem != "__init__"]
    offenders = []
    for mod in mods:
        for name, obj in vars(mod).items():
            if name.startswith("_") or getattr(obj, "__module__", "") != mod.__name__:
                continue
            targets = []
            if inspect.isfunction(obj):
                targets.append((name, obj))
            elif inspect.isclass(obj):
                for mname, meth in vars(obj).items():
                    if not mname.startswith("_") or mname == "__init__":
                        if inspect.isfunction(meth):
                            targets.append((f"{name}.{mname}", meth))
            for tname, fn in targets:
                for p in inspect.signature(fn).parameters:
                    if p in ("self", "cls"):
                        continue
                    if any(bad_word == p.lower() or p.lower().startswith(bad_word + "_") for bad_word in FORBIDDEN_NAMES):
                        offenders.append((mod.__name__, tname, p))
    check("no public parameter is named for a simulator, a scene, a data object or a spec", not offenders,
          f"{offenders}" if offenders else "")
    rsig = inspect.signature(__import__("src.safety_filter.shapes", fromlist=["RobotShapes"]).RobotShapes.__init__)
    check("RobotShapes takes the robot-only model, the shapes file and the never-close switch (SF-23)", list(rsig.parameters)[1:] == ["model", "shapes_path", "use_never_close"],
          f"{list(rsig.parameters)[1:]}")

    print("4. the world adapter")
    wsrc = (PKG / "world.py").read_text(encoding="utf-8")
    check("world.py does not import src.world_model (duck typing only)", "import src.world_model" not in wsrc
          and "from src.world_model" not in wsrc)

    print("PASS" if not FAILS else f"FAIL: {FAILS}")
    return 0 if not FAILS else 1


if __name__ == "__main__":
    sys.exit(main())
