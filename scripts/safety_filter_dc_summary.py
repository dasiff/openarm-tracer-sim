"""Summarize the never-close divide-and-conquer study and the speed-gate runs as plain Markdown tables.

Input files (paths relative to the repository root):
  data/safety_filter/never_close_dc.json              entries of the divide-and-conquer run (Section 1)
  data/safety_filter/never_close_dc_verify.json       certificate check (Section 2a)
  data/safety_filter/never_close_dc_falsify.json      falsification run (Section 2b)
  data/safety_filter/never_close_dc_replay.json       golden replay (Section 2c)
  data/safety_filter/speed_gate/<label>/gate_A.json   and gate_B.json (Sections 2d and 3)
  data/safety_filter/speed_profile/<label>.json       pair scheduler and tick times (Section 3c)

The tables go to stdout. With --out FILE the same text is also written there. A missing input prints a
"MISSING: <path>" line in its section, and the script goes on.

Usage:
    python scripts/safety_filter_dc_summary.py [--label L] [--base-label B] [--out FILE]

    --label       the run WITH the extended list (default: partb)
    --base-label  the run WITHOUT the extended list (default: partb_base)
    --out FILE    also write the text to FILE (a relative FILE is taken from the repository root)
"""

import argparse
import json
import sys
from collections import Counter
from pathlib import Path


# ----------------------------------------------------------------------------------------------------
# Constants. Every number used in this file is defined in this block, with its meaning in plain words.
# ----------------------------------------------------------------------------------------------------

TITLE = "# Never-close divide-and-conquer and speed gate summary"

# Locations.
ROOT_LEVELS_UP = 1  # folders to go up from this file to the repository root (scripts/ sits one level below it)
ROOT = Path(__file__).resolve().parents[ROOT_LEVELS_UP]
DC_PATH = "data/safety_filter/never_close_dc.json"  # Section 1: one entry per pair checked by divide and conquer
DC_VERIFY_PATH = "data/safety_filter/never_close_dc_verify.json"  # Section 2a: certificate check
DC_FALSIFY_PATH = "data/safety_filter/never_close_dc_falsify.json"  # Section 2b: falsification run
DC_REPLAY_PATH = "data/safety_filter/never_close_dc_replay.json"  # Section 2c: golden replay
SPEED_GATE_DIR = "data/safety_filter/speed_gate"  # one folder per label, holding gate_A.json and gate_B.json
SPEED_PROFILE_DIR = "data/safety_filter/speed_profile"  # one profile per label, named <label>.json
GATE_A_NAME = "gate_A.json"  # file name of the per-recording gate
GATE_B_NAME = "gate_B.json"  # file name of the closed-loop stream gate
PROFILE_SUFFIX = ".json"  # file ending of a speed profile
TEXT_ENCODING = "utf-8"  # encoding used to read input and write output
TEMP_SUFFIX = ".tmp"  # the --out file is first written under this extra ending, then renamed into place
MISSING_PREFIX = "MISSING: "  # line printed in place of a section whose input file does not exist
UNREADABLE_PREFIX = "UNREADABLE: "  # line printed when an input file exists but is not a readable JSON object
EXIT_OK = 0  # exit status on success

# Command line defaults.
DEFAULT_LABEL = "partb"  # the run WITH the extended list
DEFAULT_BASE_LABEL = "partb_base"  # the run WITHOUT the extended list

# Units, rounding and plain values.
MM_PER_M = 1000  # millimetres in one metre
PERCENT = 100  # percent in one whole; a ratio times this is a percentage
ZERO = 0  # plain zero: starting value of a count, and the guard against dividing by zero
ONE = 1  # plain one: the "- 1" in after / before - 1, and the step past one character
EXACT_VOLUME = 1.0  # a settled volume counts as settled only when it equals this exactly
UNDERSCORE = "_"  # the separator inside a shape id
MM_DECIMALS = 2  # decimals for millimetre values in general
MM_DECIMALS_ONE = 1  # decimals for the smallest search-gap margin in Section 1 (one decimal, as specified)
MM_DECIMALS_FINE = 4  # decimals for the smallest margins in Section 2a (four decimals, as specified)
SECONDS_DECIMALS = 1  # decimals for the total search time in seconds
MS_DECIMALS = 2  # decimals for timings in milliseconds
RAD_DECIMALS = 6  # decimals for joint limit overshoots and command differences in radians
PERCENT_DECIMALS = 1  # decimals for the change of a mean in percent (one decimal, as specified)
PAIRS_DECIMALS = 2  # decimals for the mean number of pairs checked per tick
TIME_KEYS = ("mean", "median", "p95", "max")  # the timing statistics shown per stream in Section 3b

# Words printed in the output.
DASH = "-"  # shown where a value is missing
NONE_WORD = "none"  # shown for an empty list of pairs
YES = "yes"  # shown for a true flag inside a table
NO = "no"  # shown for a false flag inside a table
PASS_WORD = "PASS"  # shown for a passing gate or check
FAIL_WORD = "FAIL"  # shown for a failing gate or check
SEPARATOR_CELL = " --- "  # one cell of the separator row of a Markdown table

# Pair types and result statuses (Section 1).
ARM_VS_ARM = "arm_vs_arm"
ARM_VS_BODY = "arm_vs_body"
KNOWN_KINDS = (ARM_VS_ARM, ARM_VS_BODY)  # always shown as rows, in this order
STATUSES = ("certified", "reachable", "not certified", "time cap")  # columns of Table A, in this order

# Shape ids to part names (Table C).
ARM_PREFIXES = ("left_", "right_")  # a leading one of these marks an arm shape id; it is dropped before the cut
PART_ENDINGS = ("left", "right")  # a "_" followed by a digit or by one of these ends the part name
BASE_PREFIX = "base"  # ids without an arm prefix that start with this are all shown as BASE_PART
BASE_PART = "base"  # the part name given to those ids

# Gate A counters (Section 2d).
ACTIVE_SET_KEY = "active_set_after_tick"  # the counter shown in its own column; the other counters are summed

# Table headers.
GATE_A_HEADERS = [
    "recording",
    "ticks",
    "failing ticks",
    "largest command difference rad",
    "ticks with a different active set",
    "sum of the other ticks_with_different counters",
]
REPLAY_HEADERS = [
    "recording",
    "poses",
    "pairs below threshold",
    "closest pair",
    "closest gap mm",
    "over threshold mm",
    "joint limit overshoot rad",
]
CONTROL_HEADERS = ["pair", "n_vars", "status", "threshold mm", "smallest gap found mm", "passed"]
PROFILE_HEADERS = [
    "recording",
    "pairs in the table before",
    "pairs in the table after",
    "mean pairs checked per tick before",
    "mean pairs checked per tick after",
    "pass-tick mean ms before",
    "pass-tick mean ms after",
    "acted-tick mean ms before",
    "acted-tick mean ms after",
]


# ----------------------------------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------------------------------


def load_json(path):
    """Return the JSON object in path, or None when the file is missing, unreadable or not an object."""
    if not path.is_file():
        return None
    try:
        with path.open(encoding=TEXT_ENCODING) as handle:
            data = json.load(handle)
    except (OSError, ValueError):  # ValueError also covers invalid JSON and bad text encoding
        return None
    return data if isinstance(data, dict) else None


def missing_line(path):
    """The line printed in place of a section whose input could not be loaded."""
    shown = path.relative_to(ROOT).as_posix()
    return MISSING_PREFIX + shown if not path.is_file() else UNREADABLE_PREFIX + shown


def load_or_note(lines, path):
    """Load an input file. If it cannot be loaded, add its MISSING (or UNREADABLE) line to lines and return None."""
    data = load_json(path)
    if data is None:
        lines.append(missing_line(path))
    return data


def gate_path(label, name):
    return ROOT / SPEED_GATE_DIR / label / name


def profile_path(label):
    return ROOT / SPEED_PROFILE_DIR / (label + PROFILE_SUFFIX)


def as_dict(value):
    """The value if it is a dict, else an empty dict, so that .get() calls never fail."""
    return value if isinstance(value, dict) else {}


def as_list(value):
    """The value if it is a list, else an empty list."""
    return value if isinstance(value, list) else []


def as_dicts(value):
    """The dict items of a list, in their order."""
    return [item for item in as_list(value) if isinstance(item, dict)]


def is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def is_count(value):
    return isinstance(value, int) and not isinstance(value, bool)


def numbers(values):
    """The numeric values of an iterable."""
    return [value for value in values if is_number(value)]


def total(values):
    """Sum of the numeric values; missing or non-numeric values count as zero."""
    return sum(numbers(values))


def dig(data, *keys):
    """data[key1][key2]... or None if any step is missing."""
    for key in keys:
        if not isinstance(data, dict):
            return None
        data = data.get(key)
    return data


def diff(item, minuend, subtrahend):
    """item[minuend] minus item[subtrahend], or None if either one is missing or not a number."""
    first = item.get(minuend)
    second = item.get(subtrahend)
    if is_number(first) and is_number(second):
        return first - second
    return None


def fmt(value, decimals=None):
    """Text for a table cell: DASH for None, yes/no for booleans, fixed decimals for numbers when asked."""
    if value is None:
        return DASH
    if isinstance(value, bool):
        return YES if value else NO
    if decimals is not None and is_number(value):
        return f"{value:.{decimals}f}"
    return str(value)


def mm_text(metres, decimals):
    """A length given in metres, shown in millimetres with the given number of decimals."""
    if not is_number(metres):
        return DASH
    return f"{metres * MM_PER_M:.{decimals}f}"


def pct_change(before, after):
    """Change from before to after in percent with a sign. DASH if either is missing or before is zero."""
    if not (is_number(before) and is_number(after)) or before == ZERO:
        return DASH
    return f"{(after / before - ONE) * PERCENT:+.{PERCENT_DECIMALS}f}%"


def pair_text(pair):
    """A pair of shape ids as 'a / b'."""
    if isinstance(pair, list):
        return " / ".join(str(item) for item in pair)
    return fmt(pair)


def id_list_text(pairs):
    """A list of pairs as 'a / b, c / d', or NONE_WORD when the list is empty or missing."""
    return ", ".join(pair_text(pair) for pair in as_list(pairs)) or NONE_WORD


def count_text(value):
    """Number of items in a list, or DASH when the key is missing."""
    if value is None:
        return DASH
    return str(len(as_list(value)))


def verdict(flag):
    return PASS_WORD if flag is True else FAIL_WORD


def kind_of(entry):
    return str(entry.get("pair_type"))


def cut_part(text):
    """Text before the first underscore that is followed by a digit, 'left' or 'right'. The whole text if none."""
    start = ZERO
    while True:
        index = text.find(UNDERSCORE, start)
        if index < ZERO:
            return text
        tail = text[index + ONE:]
        if tail[:ONE].isdigit() or tail.startswith(PART_ENDINGS):
            return text[:index]
        start = index + ONE


def part_of(shape_id):
    """Part name of a shape id, as used in Table C."""
    if not isinstance(shape_id, str):
        return DASH
    for prefix in ARM_PREFIXES:
        if shape_id.startswith(prefix):
            return cut_part(shape_id[len(prefix):])
    if shape_id.startswith(BASE_PREFIX):
        return BASE_PART
    return shape_id


def ordered_names(first, second):
    """Names of first in their order, then the names that only appear in second."""
    return list(first) + [name for name in second if name not in first]


def recordings_of(data):
    """The 'recordings' object of a result file, or an empty dict."""
    return as_dict(dig(data, "recordings"))


def md_table(headers, rows):
    """Markdown table lines for a list of header names and a list of rows."""
    lines = ["| " + " | ".join(headers) + " |", "|" + "|".join(SEPARATOR_CELL for _ in headers) + "|"]
    for row in rows:
        lines.append("| " + " | ".join(str(cell) for cell in row) + " |")
    return lines


def write_text_atomic(path, text):
    """Write text to path through a temporary file, so an interrupted run leaves the old file intact."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + TEMP_SUFFIX)
    temp.write_text(text, encoding=TEXT_ENCODING)
    temp.replace(path)


# ----------------------------------------------------------------------------------------------------
# Sections
# ----------------------------------------------------------------------------------------------------


def section_pairs(lines):
    """Section 1: pairs by result and pair type, listed pairs by number of variables and by part pair."""
    lines.append("## 1. Pairs by result and type")
    lines.append("")
    data = load_or_note(lines, ROOT / DC_PATH)
    if data is None:
        lines.append("")
        return
    entries = as_dicts(data.get("entries"))
    listed = [entry for entry in entries if entry.get("listed") is True]

    # Table A: result counts per pair type, with a total row.
    kinds = list(KNOWN_KINDS) + sorted({kind_of(entry) for entry in entries} - set(KNOWN_KINDS))
    rows = []
    numeric_rows = []
    for kind in kinds:
        of_kind = [entry for entry in entries if kind_of(entry) == kind]
        counts = [len([entry for entry in of_kind if entry.get("status") == status]) for status in STATUSES]
        counts.append(len(of_kind))
        numeric_rows.append(counts)
        rows.append([kind] + counts)
    rows.append(["total"] + [sum(column) for column in zip(*numeric_rows)])
    lines.append("### Table A. Result by pair type")
    lines.append("")
    lines.extend(md_table(["pair type"] + list(STATUSES) + ["total"], rows))
    lines.append("")

    # Table B: listed (certified) pairs by number of variables.
    n_vars_values = sorted(value for value in {entry.get("n_vars") for entry in listed} if value is not None)
    if any(entry.get("n_vars") is None for entry in listed):
        n_vars_values.append(None)
    rows = []
    for n_vars in n_vars_values:
        same = [entry for entry in listed if entry.get("n_vars") == n_vars]
        rows.append([
            fmt(n_vars),
            len([entry for entry in same if kind_of(entry) == ARM_VS_ARM]),
            len([entry for entry in same if kind_of(entry) == ARM_VS_BODY]),
            len(same),
        ])
    lines.append("### Table B. Listed (certified) pairs by number of variables")
    lines.append("")
    lines.extend(md_table(["n_vars", ARM_VS_ARM, ARM_VS_BODY, "total"], rows))
    lines.append("")

    # Table C: listed pairs by part pair (each element of shape_pairs is one pair).
    part_names = []
    for entry in listed:
        for pair in as_list(entry.get("shape_pairs")):
            if isinstance(pair, list):
                part_names.append(" vs ".join(sorted(part_of(shape_id) for shape_id in pair)))
    part_counts = Counter(part_names)
    rows = [[name, part_counts[name]] for name in sorted(part_counts)]
    lines.append("### Table C. Listed pairs by part pair")
    lines.append("")
    lines.extend(md_table(["part pair", "count"], rows))
    lines.append("")

    seconds = total(entry.get("seconds") for entry in entries)
    boxes = total(entry.get("boxes_tested") for entry in entries)
    margins = numbers(diff(entry, "search_gap_m", "needed_m") for entry in listed)
    smallest = min(margins) if margins else None
    lines.append(
        f"Total seconds over all {len(entries)} entries: {fmt(seconds, SECONDS_DECIMALS)}. "
        f"Total boxes tested: {fmt(boxes)}. "
        f"Smallest search_gap_m - needed_m over listed pairs: {mm_text(smallest, MM_DECIMALS_ONE)} mm."
    )
    lines.append("")


def section_certificate(lines):
    """Section 2a: the certificate check of the divide-and-conquer run."""
    lines.append("### 2a. Certificate check")
    lines.append("")
    data = load_or_note(lines, ROOT / DC_VERIFY_PATH)
    if data is None:
        lines.append("")
        return
    pairs = as_dicts(data.get("pairs"))
    not_same = len([pair for pair in pairs if pair.get("same_as_first_run") is False])
    poses = total(pair.get("poses_rechecked") for pair in pairs)
    filter_margins = numbers(pair.get("min_filter_gap_minus_bound_m") for pair in pairs)
    bound_margins = numbers(diff(pair, "min_lower_bound_m", "needed_m") for pair in pairs)
    not_exact = len([pair for pair in pairs if pair.get("settled_volume") != EXACT_VOLUME])
    smallest_filter = min(filter_margins) if filter_margins else None
    smallest_bound = min(bound_margins) if bound_margins else None
    lines.append(
        f"certificate check: {fmt(data.get('n_passed'))} of {fmt(data.get('n_pairs'))} passed; "
        f"failed: {id_list_text(data.get('failed'))}; "
        f"pairs with same_as_first_run false: {not_same}; "
        f"total poses_rechecked: {fmt(poses)}; "
        f"smallest min_filter_gap_minus_bound_m: {mm_text(smallest_filter, MM_DECIMALS_FINE)} mm; "
        f"smallest (min_lower_bound_m - needed_m): {mm_text(smallest_bound, MM_DECIMALS_FINE)} mm; "
        f"pairs whose settled_volume is not exactly {EXACT_VOLUME}: {not_exact}."
    )
    lines.append("")


def section_falsify(lines):
    """Section 2b: the falsification run, declared pairs on one line and the control pairs as a table."""
    lines.append("### 2b. Falsification run")
    lines.append("")
    data = load_or_note(lines, ROOT / DC_FALSIFY_PATH)
    if data is None:
        lines.append("")
        return
    declared = as_dicts(data.get("declared"))
    evaluations = total(row.get("evaluations") for row in declared)
    lines.append(
        f"declared pairs: {fmt(data.get('n_declared_passed'))} of {fmt(data.get('n_declared'))} passed; "
        f"failed: {id_list_text(data.get('declared_failed'))}; "
        f"restarts: {fmt(data.get('restarts'))}; "
        f"smallest margin over threshold: {mm_text(data.get('smallest_margin_over_threshold_m'), MM_DECIMALS)} mm; "
        f"total evaluations: {fmt(evaluations)}."
    )
    lines.append("")
    lines.append(f"control pairs: {fmt(data.get('n_control_passed'))} of {fmt(data.get('n_control'))} passed.")
    lines.append("")
    rows = []
    for control in as_dicts(data.get("control")):
        rows.append([
            pair_text(control.get("pair")),
            fmt(control.get("n_vars")),
            fmt(control.get("status")),
            mm_text(control.get("threshold_m"), MM_DECIMALS),
            mm_text(control.get("min_gap_m"), MM_DECIMALS),
            fmt(control.get("passed")),
        ])
    lines.extend(md_table(CONTROL_HEADERS, rows))
    lines.append("")


def section_replay(lines):
    """Section 2c: the golden replay, one row per recording."""
    lines.append("### 2c. Golden replay")
    lines.append("")
    data = load_or_note(lines, ROOT / DC_REPLAY_PATH)
    if data is None:
        lines.append("")
        return
    rows = []
    for rec in as_dicts(data.get("recordings")):
        rows.append([
            fmt(rec.get("recording")),
            fmt(rec.get("poses")),
            count_text(rec.get("pairs_below_threshold")),
            pair_text(rec.get("closest_pair")),
            mm_text(rec.get("closest_gap_m"), MM_DECIMALS),
            mm_text(rec.get("closest_gap_minus_threshold_m"), MM_DECIMALS),
            fmt(rec.get("largest_joint_limit_overshoot_rad"), RAD_DECIMALS),
        ])
    lines.extend(md_table(REPLAY_HEADERS, rows))
    lines.append("")
    lines.append(f"golden replay: {verdict(data.get('passed'))}")
    lines.append("")


def section_gate(lines, label):
    """Section 2d: speed gate A (per recording) and gate B (per stream) for one label."""
    lines.append(f"### 2d. Speed gate, label {label}")
    lines.append("")

    gate_a = load_or_note(lines, gate_path(label, GATE_A_NAME))
    if gate_a is not None:
        rows = []
        for name, rec in recordings_of(gate_a).items():
            rec = as_dict(rec)
            diffs = rec.get("ticks_with_different")
            counters = as_dict(diffs)
            other = sum(value for key, value in counters.items() if key != ACTIVE_SET_KEY and is_count(value))
            rows.append([
                name,
                fmt(rec.get("ticks")),
                fmt(rec.get("failing_ticks")),
                fmt(rec.get("largest_cmd_diff_rad"), RAD_DECIMALS),
                fmt(counters.get(ACTIVE_SET_KEY)),
                fmt(other) if isinstance(diffs, dict) else DASH,
            ])
        lines.extend(md_table(GATE_A_HEADERS, rows))
        lines.append("")
        lines.append(f"gate A: {verdict(gate_a.get('pass'))}")
    lines.append("")

    gate_b = load_or_note(lines, gate_path(label, GATE_B_NAME))
    if gate_b is not None:
        rows = []
        for name, stream in as_dict(gate_b.get("streams")).items():
            stream = as_dict(stream)
            rows.append([
                name,
                fmt(stream.get("pass")),
                fmt(stream.get("identical_to_golden_commands_and_status")),
                fmt(stream.get("first_tick_with_a_different_command")),
            ])
        lines.extend(md_table(["stream", "pass", "identical to golden", "first different tick"], rows))
        lines.append("")
        lines.append(f"gate B: {verdict(gate_b.get('pass'))}")
    lines.append("")


def section_soundness(lines, label):
    """Section 2: soundness checks (2a to 2d)."""
    lines.append("## 2. Soundness checks")
    lines.append("")
    section_certificate(lines)
    section_falsify(lines)
    section_replay(lines)
    section_gate(lines, label)


def section_timing(lines, label, base_label):
    """Section 3: timing and pairs checked, before (base label) and after (label)."""
    lines.append("## 3. Timing and pairs checked, before and after")
    lines.append("")
    lines.append(f"before = {base_label}, after = {label}. Times are in milliseconds.")
    lines.append("")

    # 3a: gate A mean and p95 per recording, with the change of the mean.
    lines.append("### 3a. Gate A timing per recording")
    lines.append("")
    before = load_or_note(lines, gate_path(base_label, GATE_A_NAME))
    after = load_or_note(lines, gate_path(label, GATE_A_NAME))
    before_recs = recordings_of(before)
    after_recs = recordings_of(after)
    rows = []
    for name in ordered_names(after_recs, before_recs):
        before_time = as_dict(as_dict(before_recs.get(name)).get("time_ms"))
        after_time = as_dict(as_dict(after_recs.get(name)).get("time_ms"))
        rows.append([
            name,
            fmt(before_time.get("mean"), MS_DECIMALS),
            fmt(before_time.get("p95"), MS_DECIMALS),
            fmt(after_time.get("mean"), MS_DECIMALS),
            fmt(after_time.get("p95"), MS_DECIMALS),
            pct_change(before_time.get("mean"), after_time.get("mean")),
        ])
    lines.extend(md_table(
        ["recording", "before mean (ms)", "before p95 (ms)", "after mean (ms)", "after p95 (ms)", "change of mean"],
        rows,
    ))
    lines.append("")

    # 3b: closed loop timing per stream, for the label only.
    lines.append(f"### 3b. Closed loop timing per stream, {label}")
    lines.append("")
    gate_b = load_or_note(lines, gate_path(label, GATE_B_NAME))
    if gate_b is not None:
        rows = []
        for name, stream in as_dict(gate_b.get("streams")).items():
            time_ms = as_dict(as_dict(stream).get("time_ms"))
            rows.append([name] + [fmt(time_ms.get(key), MS_DECIMALS) for key in TIME_KEYS])
        lines.extend(md_table(["stream", "mean (ms)", "median (ms)", "p95 (ms)", "max (ms)"], rows))
    lines.append("")

    # 3c: pair scheduler and tick times from the speed profiles.
    lines.append("### 3c. Pair scheduler and tick times from the profiles")
    lines.append("")
    before = load_or_note(lines, profile_path(base_label))
    after = load_or_note(lines, profile_path(label))
    before_recs = recordings_of(before)
    after_recs = recordings_of(after)
    rows = []
    for name in ordered_names(after_recs, before_recs):
        b = as_dict(before_recs.get(name))
        a = as_dict(after_recs.get(name))
        rows.append([
            name,
            fmt(dig(b, "pair_scheduler", "pairs")),
            fmt(dig(a, "pair_scheduler", "pairs")),
            fmt(dig(b, "pair_scheduler", "mean_pairs_checked_per_tick"), PAIRS_DECIMALS),
            fmt(dig(a, "pair_scheduler", "mean_pairs_checked_per_tick"), PAIRS_DECIMALS),
            fmt(dig(b, "classes", "pass", "tick_ms_mean_with_timers"), MS_DECIMALS),
            fmt(dig(a, "classes", "pass", "tick_ms_mean_with_timers"), MS_DECIMALS),
            fmt(dig(b, "classes", "acted", "tick_ms_mean_with_timers"), MS_DECIMALS),
            fmt(dig(a, "classes", "acted", "tick_ms_mean_with_timers"), MS_DECIMALS),
        ])
    lines.extend(md_table(PROFILE_HEADERS, rows))
    lines.append("")


def main():
    parser = argparse.ArgumentParser(
        description="Summarize the never-close divide-and-conquer study and the speed gate as Markdown tables.")
    parser.add_argument("--label", default=DEFAULT_LABEL,
                        help="the run WITH the extended list (default: %(default)s)")
    parser.add_argument("--base-label", default=DEFAULT_BASE_LABEL,
                        help="the run WITHOUT the extended list (default: %(default)s)")
    parser.add_argument("--out", default=None,
                        help="also write the text to this file (a relative path is taken from the repository root)")
    args = parser.parse_args()

    lines = [TITLE, "", f"Run WITH the extended list: {args.label}. Run WITHOUT it: {args.base_label}.", ""]
    section_pairs(lines)
    section_soundness(lines, args.label)
    section_timing(lines, args.label, args.base_label)
    text = "\n".join(lines) + "\n"
    sys.stdout.write(text)
    if args.out:
        write_text_atomic(ROOT / args.out, text)
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
