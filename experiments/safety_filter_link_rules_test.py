"""Safety filter Stage 2: link categories (SF-28) and the config explanation test (every Stage 2 setting has a plain-word comment and a decision row).

Run: python experiments/safety_filter_link_rules_test.py   (exit 0 = pass). No simulation.
"""

import ast
import json
import re
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.link_rules import CATEGORIES, category_table  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(f"  {'ok  ' if cond else 'FAIL'} {name} {detail}")
    if not cond:
        FAILS.append(name)


def main():
    doc = json.loads((ROOT / cfg.SHAPES_FILE).read_text(encoding="utf-8"))
    rows = category_table(doc["shapes"])
    print("capsule -> category table")
    for r in rows:
        print(f"  {r[0]:22s} {r[1]:6s} {r[2]:15s} {r[3]}")
    arm_caps = [s for s in doc["shapes"] if s["kind"] == "capsule" and s["arm"] in ("left", "right")]
    check("every arm capsule has exactly one category", len(rows) == len(arm_caps) and len({r[0] for r in rows}) == len(rows)
          and all(r[3] in CATEGORIES for r in rows), f"({len(rows)} capsules)")
    check("every arm capsule is classified by the table (none falls back to shoulder)", all(r[4] for r in rows),
          f"unclassified: {[r[0] for r in rows if not r[4]]}")
    by = {}
    for r in rows:
        by.setdefault(r[2], set()).add(r[3])
    check("links 1,2 shoulder; 3,4 upper_arm; 5,6 forearm; 7, palm, finger hand",
          by["link1"] == by["link2"] == {"shoulder"} and by["link3"] == by["link4"] == {"upper_arm"}
          and by["link5"] == by["link6"] == {"forearm"} and by["link7"] == by["palm"] == by["finger"] == {"hand"})
    print("  counts:", dict(Counter(r[3] for r in rows)))

    print("config explanation test")
    src = (ROOT / "src/safety_filter/config.py").read_text(encoding="utf-8")
    lines = src.splitlines()
    tree = ast.parse(src)
    ref = re.compile(r"\[[^\]]*(SF-\d+|M\d+)[^\]]*\]")
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") in ("RESOLVER", "GAP_RULE"):
            name = node.targets[0].id
            for k in node.value.keys:
                line = lines[k.lineno - 1]
                comment = line.split("#", 1)[1].strip() if "#" in line else ""
                has_ref = bool(ref.search(comment))
                words = len(re.sub(r"\[[^\]]*\]", "", comment).split())
                check(f"{name}[{k.value!r}] has a comment (>= 5 words) and a row reference", words >= 5 and has_ref)
                if name == "RESOLVER":
                    check(f"{name}[{k.value!r}] names an SF row", "SF-" in comment)
    print("PASS" if not FAILS else f"FAIL: {FAILS}")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
