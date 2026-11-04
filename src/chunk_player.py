"""Chunk player (David, Nov 4, 2026; decision row M253). A source may return a chunk: {"mode": "chunk", "commands": [...]}, a batch of future
commands (a VLA returns 50 at 30 per second). The control loop plays the first CHUNK_CFG["play_count"] of them, one per due tick at the
policy rate (CHUNK_CFG["rate_hz"], here every 5 ticks at 150 Hz), and then asks the source again. Any other output (a command dict, a list of
dicts, or None) is handled exactly as before; a list keeps its old meaning (applied at once, last command wins).
"""

from src.task_config import CHUNK_CFG


def is_chunk(out):
    """True if a source's output is a chunk (a dict with mode "chunk")."""
    return isinstance(out, dict) and out.get("mode") == "chunk"


class ChunkPlayer:
    """Plays one chunk. next(frame) returns the command for this tick (on a due tick), or None (not due, or the chunk is used up)."""

    def __init__(self, commands, frame, period_ticks, play_count=None):
        self.commands = list(commands)
        self.start = frame
        self.period = max(1, int(period_ticks))
        self.play_count = CHUNK_CFG["play_count"] if play_count is None else int(play_count)
        self.index = 0
        self.played = 0

    def has_more(self):
        """True while the loop should play from this chunk (the source is not asked)."""
        return self.played < min(self.play_count, len(self.commands))

    def next(self, frame):
        if not self.has_more() or (frame - self.start) % self.period:
            return None
        cmd = self.commands[self.index]
        self.index += 1
        self.played += 1
        return cmd

    def discard(self):
        """Drops the rest of the chunk (the open catch does this; decision 4)."""
        self.played = self.play_count
