"""Compare pre/baseline vs post_head episode_log.csv files."""
import csv
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent.parent
pre_baseline = ROOT / "data" / "diag" / "logging_20261101" / "pre" / "baseline" / "episode" / "episode_log.csv"
post_head = ROOT / "data" / "diag" / "logging_20261101" / "post_head" / "episode" / "episode_log.csv"

def load_csv(path):
    """Load CSV and return (headers, rows as list of dicts)."""
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    return reader.fieldnames, rows

print("Loading files...")
pre_headers, pre_rows = load_csv(pre_baseline)
post_headers, post_rows = load_csv(post_head)

print(f"\nPre/baseline tick count: {len(pre_rows)}")
print(f"Post_head tick count: {len(post_rows)}")

print(f"\nPre/baseline columns ({len(pre_headers)}):")
print(pre_headers)

print(f"\nPost_head columns ({len(post_headers)}):")
print(post_headers)

pre_cols = set(pre_headers)
post_cols = set(post_headers)
common_cols = pre_cols & post_cols
only_pre = pre_cols - post_cols
only_post = post_cols - pre_cols

print(f"\nColumns in both: {len(common_cols)}")
print(f"Only in pre/baseline: {sorted(only_pre)}")
print(f"Only in post_head: {sorted(only_post)}")

print("\n=== COMPARISON OF COMMON COLUMNS ===")
for col in sorted(common_cols):
    try:
        first_diff_tick = None
        max_diff = 0.0
        identical = True

        for i, (pre_row, post_row) in enumerate(zip(pre_rows, post_rows)):
            try:
                pre_val = float(pre_row[col]) if pre_row[col] else float('nan')
                post_val = float(post_row[col]) if post_row[col] else float('nan')
                diff = abs(pre_val - post_val)

                if diff > 1e-9:
                    identical = False
                    if first_diff_tick is None:
                        first_diff_tick = i
                    max_diff = max(max_diff, diff)
            except (ValueError, TypeError):
                if pre_row[col] != post_row[col]:
                    identical = False
                    if first_diff_tick is None:
                        first_diff_tick = i

        if identical:
            print(f"{col}: identical")
        else:
            print(f"{col}: max_diff={max_diff:.9e}, first_diff_tick={first_diff_tick}")
    except Exception as e:
        print(f"{col}: ERROR - {e}")

print("\n=== DONE ===")
