"""Recording of the filter's input and output, every tick (M296, SF-37). Follows the layout of src/run_record.py:

    run_dir/
        meta.json              format, complete flag, tick length, joint order, status names and hold reasons, started/finished
        chunks/chunk_00000.npz ... ticks [start, start + n): start, raw[n, 2, 7], filtered[n, 2, 7] (rad, float64; left arm, right arm),
                               status[n] (index into meta "statuses"), hold_reason[n] (index into meta "hold_reasons", -1 none),
                               rounds[n], worst_gap[n] (m, nan if none), worst_shortfall[n] (m), fraction[n] (nan if none),
                               worst_pair[n] (text "idA|idB", empty if none), methods[n] (text, comma separated)
Chunks are written every `chunk_ticks` ticks, so a crash loses at most the last chunk. A chunk is written to a temporary
file and renamed (no half-written file). Which of raw and filtered is "the action" is not decided here (M296: the control-loop stage).
"""

import json
import time
from pathlib import Path

import numpy as np

from src.safety_filter.resolver import HOLD_REASONS, STATUSES


class FilterRecorder:
    def __init__(self, run_dir, dt, chunk_ticks=2400):
        """run_dir: directory to write into (created). dt (s): the tick length. chunk_ticks: ticks per chunk file."""
        self.dir = Path(run_dir)
        (self.dir / "chunks").mkdir(parents=True, exist_ok=True)
        self.chunk_ticks, self.n_chunks, self.start_tick, self.n_ticks = chunk_ticks, 0, 0, 0
        self.meta = {"format": 1, "complete": False, "started": time.strftime("%Y-%m-%d %H:%M:%S"), "dt": float(dt),
                     "joint_order": "left joints 1-7, right joints 1-7 (array [tick, arm, joint])",
                     "statuses": list(STATUSES), "hold_reasons": list(HOLD_REASONS)}
        self._reset()
        self._write_meta()

    def _reset(self):
        self.raw, self.filtered, self.rows = [], [], []

    def _write_meta(self):
        tmp = self.dir / "meta.json.tmp"
        tmp.write_text(json.dumps({**self.meta, "n_ticks": self.n_ticks}, indent=1))
        tmp.replace(self.dir / "meta.json")

    def tick(self, raw, filtered, status):
        """Records one tick: the raw command, the filtered command and the FilterStatus (copied)."""
        self.raw.append(np.array(raw, dtype=np.float64))
        self.filtered.append(np.array(filtered, dtype=np.float64))
        self.rows.append(status)
        self.n_ticks += 1
        if len(self.raw) >= self.chunk_ticks:
            self._flush()

    def _flush(self):
        if not self.raw:
            return
        r = self.rows
        name = self.dir / "chunks" / f"chunk_{self.n_chunks:05d}.npz"
        tmp = self.dir / "chunks" / f"chunk_{self.n_chunks:05d}.tmp.npz"
        np.savez(tmp, start=self.start_tick, raw=np.array(self.raw), filtered=np.array(self.filtered),
                 status=np.array([STATUSES.index(s.status) for s in r], dtype=np.int8),
                 hold_reason=np.array([HOLD_REASONS.index(s.hold_reason) if s.hold_reason else -1 for s in r], dtype=np.int8),
                 rounds=np.array([s.rounds for s in r], dtype=np.int16),
                 worst_gap=np.array([s.worst_gap for s in r], dtype=np.float64),
                 worst_shortfall=np.array([s.worst_shortfall for s in r], dtype=np.float64),
                 fraction=np.array([s.fraction for s in r], dtype=np.float64),
                 worst_pair=np.array(["|".join(s.worst_pair) if s.worst_pair else "" for s in r]),
                 methods=np.array([",".join(s.methods) for s in r]))
        tmp.replace(name)
        self.start_tick += len(self.raw)
        self.n_chunks += 1
        self._reset()
        self._write_meta()

    def close(self):
        """Writes the last chunk and marks the recording complete."""
        self._flush()
        self.meta.update(complete=True, finished=time.strftime("%Y-%m-%d %H:%M:%S"))
        self._write_meta()


def load_filter_recording(run_dir):
    """Reads a recording: dict with meta and the arrays of all chunks joined (raw, filtered, status (names), hold_reason (names or ''), ...)."""
    run_dir = Path(run_dir)
    meta = json.loads((run_dir / "meta.json").read_text())
    files = sorted((run_dir / "chunks").glob("chunk_?????.npz"))
    if not files:
        raise ValueError(f"no chunks in {run_dir}")
    parts = [np.load(f) for f in files]
    out = {k: np.concatenate([p[k] for p in parts]) for k in parts[0].files if k != "start"}
    out["status"] = np.array([meta["statuses"][i] for i in out["status"]])
    out["hold_reason"] = np.array([meta["hold_reasons"][i] if i >= 0 else "" for i in out["hold_reason"]])
    out["meta"] = meta
    return out
