"""Catch the open command (David's decisions of Nov 4, 2026; decision row M252). Off unless OPEN_CATCH_CFG["enabled"] is True.

Why: a VLA sends its commands as a batch (a chunk). Nothing announces the moment between lowering onto the hotplate and opening the
gripper. Without this helper, an open command would open the gripper while the arm still carries the beaker's weight in its feedforward
(hand bounce). With it:
1. the open does not reach the gripper, which stays closed;
2. while waiting, the whole arm and gripper command stream is held: no new command is applied, the last targets are sent again;
3. the same held-mass ramp-out runs (held_mass_ramp.set_target to 0, the same duration as today), with the same sag-trim freeze
   (the sag trim is not integrated during the hold, and the held-mass ramp keeps it frozen);
4. when the ramp finishes, the open is released and the gripper opens through the existing gripper-open wait (the caller's code);
5. the commands after the open in the chunk are kept and played in their original order after the gripper has finished opening;
6. a second open while a hold is running is the same request: it does not restart the ramp.

Nothing here re-implements the ramp or the freeze: the caller passes in the existing functions (see src/mpvi_modules.py, open branch).
"""


class OpenCatch:
    """One open request and its hold. States: "idle" (no request), "holding" (ramp running, stream held), "released" (ramp finished)."""

    def __init__(self, start_ramp, ramp_done):
        """start_ramp(t, tick): starts the held-mass ramp to zero (the existing held_mass_ramp.set_target). ramp_done(t) -> bool: the ramp has finished (the existing held_mass_ramp.finished)."""
        self.start_ramp = start_ramp
        self.ramp_done = ramp_done
        self.state = "idle"
        self.requests = 0          # every open request seen, including repeats during the hold
        self.ignored = 0           # repeats during a hold (decision 6)

    def request(self, t, tick):
        """An open command arrives. Starts the hold if idle. Returns True if it started the hold, False if it was a repeat (no restart)."""
        self.requests += 1
        if self.state != "idle":
            self.ignored += 1
            return False
        self.state = "holding"
        self.start_ramp(t, tick)
        return True

    def ramp_finished(self, t):
        """True once the ramp has finished; the open is then released (state "released"). Stays True afterwards."""
        if self.state == "holding" and self.ramp_done(t):
            self.state = "released"
        return self.state == "released"


class ChunkPlayer:
    """Plays a stream of commands for the synthetic tests (src/open_catch test in experiments/open_catch_test.py).

    Items are ("cmd", payload) or ("open", None). One item plays per tick. While an open is held, nothing else plays: the held
    command (the last arm command, gripper closed) is returned. After the ramp, the open is released and the gripper_opened(t)
    callback is awaited; only then do the remaining items play, in order.
    """

    def __init__(self, items, gripper_opened, start_ramp, ramp_done):
        """items: the chunk. gripper_opened(t) -> bool: the existing gripper-open wait has finished. start_ramp and ramp_done: the
        same two functions OpenCatch takes (the existing held-mass ramp functions in the real code)."""
        self.queue = list(items)
        self.gripper_opened = gripper_opened
        self.start_ramp = start_ramp
        self.ramp_done = ramp_done
        self.catch = None
        self.last_arm = None

    def send(self, item, t, tick):
        """An item arrives from the stream. An open while a hold runs is a repeat: it is passed to the running hold and ignored there."""
        if item[0] == "open" and self.catch is not None:
            self.catch.request(t, tick)
            return
        self.queue.append(item)

    def tick(self, t, tick):
        """Returns (kind, payload) for this tick: ("cmd", arm command), ("hold", held arm command), ("open", None) while the gripper
        is being opened, or ("none", None) when nothing is left."""
        if self.catch is not None:
            if not self.catch.ramp_finished(t):
                return ("hold", self.last_arm)
            if not self.gripper_opened(t):
                return ("open", None)
            self.catch = None
        if not self.queue:
            return ("none", None)
        kind, payload = self.queue.pop(0)
        if kind == "open":
            self.catch = OpenCatch(self.start_ramp, self.ramp_done)
            self.catch.request(t, tick)
            return self.tick(t, tick)
        self.last_arm = payload
        return ("cmd", payload)
