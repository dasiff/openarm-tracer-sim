"""Reads and updates experiments/baselines/INDEX.csv, the list of which baseline file is the CURRENT one for each bench scenario.

Columns: scenario, file (relative to experiments/baselines/), date, commit, status, reason, sha256, clean_code.
Statuses: CURRENT (the one the tools compare against), RETIRED (kept for history, in experiments/baselines/retired/),
NO_CURRENT (the scenario has no baseline right now; the tools must stop).

Command line (used by experiments/cc_bench_regen.sh; read-only except "promote"):
    python experiments/cc_bench_index.py current <scenario>
        prints the CURRENT file's path (relative to experiments/baselines/); exit 3 if NO_CURRENT, 4 if no row, 2 if the index is missing
    python experiments/cc_bench_index.py promote <scenario> <file> <date> <commit> <sha256> <clean_code> <reason>
        marks the old CURRENT row RETIRED (its file moved to retired/ beforehand with git mv), removes the NO_CURRENT row, adds the new CURRENT row
"""

import csv
import os
import sys

BASELINES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "baselines")
INDEX = os.path.join(BASELINES, "INDEX.csv")
COLUMNS = ["scenario", "file", "date", "commit", "status", "reason", "sha256", "clean_code"]


class IndexMissing(Exception):
    pass


class NoCurrent(Exception):
    pass


class NoRow(Exception):
    pass


def read(path=INDEX):
    if not os.path.isfile(path):
        raise IndexMissing(path)
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if rows and list(rows[0].keys()) != COLUMNS:
        raise IndexMissing(f"{path} has columns {list(rows[0].keys())}, expected {COLUMNS}")
    return rows


def write(rows, path=INDEX):
    tmp = path + ".tmp"
    with open(tmp, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(rows)
    os.replace(tmp, path)


def current(scenario, path=INDEX):
    """The CURRENT file of a scenario, relative to experiments/baselines/. Raises NoCurrent or NoRow; never guesses."""
    rows = read(path)
    mine = [r for r in rows if r["scenario"] == scenario]
    if not mine:
        raise NoRow(f"no row for scenario '{scenario}' in {path}")
    cur = [r for r in mine if r["status"] == "CURRENT"]
    if len(cur) == 1:
        return cur[0]["file"]
    if len(cur) > 1:
        raise NoCurrent(f"more than one CURRENT row for '{scenario}' in {path}")
    raise NoCurrent(f"scenario '{scenario}' has status {sorted({r['status'] for r in mine})} in {path}: no current baseline")


def promote(scenario, file, date, commit, sha256, clean_code, reason, path=INDEX):
    """Retires the old CURRENT row of the scenario (its file is now under retired/), drops a NO_CURRENT row, adds the new CURRENT row."""
    rows = read(path)
    if any(r["file"] == file for r in rows):
        raise ValueError(f"{file} already has a row in {path}")
    out = []
    retired = None
    for r in rows:
        if r["scenario"] == scenario and r["status"] == "NO_CURRENT":
            continue
        if r["scenario"] == scenario and r["status"] == "CURRENT":
            r = dict(r, status="RETIRED", file="retired/" + os.path.basename(r["file"]),
                     reason=f"retired {date} by the regeneration of {file}")
            retired = r["file"]
        out.append(r)
    out.append({"scenario": scenario, "file": file, "date": date, "commit": commit, "status": "CURRENT",
                "reason": reason, "sha256": sha256, "clean_code": clean_code})
    write(out, path)
    return retired


def main(argv):
    if len(argv) >= 2 and argv[0] == "current":
        try:
            print(current(argv[1]))
            return 0
        except IndexMissing as e:
            print(f"baseline index missing or unreadable: {e}", file=sys.stderr)
            return 2
        except NoCurrent as e:
            print(f"no current baseline: {e}", file=sys.stderr)
            return 3
        except NoRow as e:
            print(f"no row: {e}", file=sys.stderr)
            return 4
    if len(argv) == 8 and argv[0] == "promote":
        old = promote(*argv[1:8])
        print(f"promoted {argv[2]}; retired row: {old}")
        return 0
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
