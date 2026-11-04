"""Catch the open command, in the control loop, one object per hand (decisions M253, David, Nov 4, 2026). Always on: there is no switch.

A VLA sends "open" as one command; nothing announces the step between lowering onto the hotplate and opening the gripper. The loop
checks every open command for one hand:
1. If the hand is NOT holding (the holding verdict, src/holding.py), the open passes straight through with no delay.
2. If the hand IS holding, the open is caught: it does not reach the gripper (the gripper stays closed), the rest of the source's
   current chunk is discarded, and the source is not asked for new commands during the hold.
3. The hold runs the existing held-mass ramp-out (held_mass_ramp.set_target to 0 and held_mass_ramp.finished, the same 1.0 s as
   before) with the existing sag-trim freeze (the sag trim does not integrate while a held-mass ramp runs, mpvi_modules _sag_trim_step).
4. When the ramp finishes, the open is released and the gripper opens; the stop test below runs (the same test the script's
   close and open steps use). When the fingers have stopped open, the source is asked for fresh commands.
5. A repeat of the same open during the hold is ignored and logged (the ramp is not restarted).
6. If the holding verdict goes off during the ramp (the grasp failed), the hold is cancelled, the source's commands pass through at
   once, and a "failed_hold" entry is logged. The caught open is not sent.
7. A second, different open during the hold: the first is kept, the difference is ignored and logged.

Numbers: the ramp is PICK_PLACE_CFG["held_mass_ramp_s"]; the stop test uses PICK_PLACE_CFG stall_speed_m_s, stall_dwell_s,
open_reached_margin_m; the open command threshold is OPEN_CATCH_CFG["open_command_min_m"] (PROVISIONAL).
"""

from src import held_mass_ramp as hmr
from src.chunk_player import ChunkPlayer, is_chunk
from src.task_config import OPEN_CATCH_CFG, PICK_PLACE_CFG, TASK_CFG


def stall_done(travel, speed, still, t, closing, cfg, open_g):
    """The gripper-stop test shared by the script's close and open steps and by the catch: the fingers are slower than stall_speed_m_s
    for stall_dwell_s, and, for an open, have reached the open position (within open_reached_margin_m). Returns (done, still), where
    still is the time the fingers first stopped (None if they are moving)."""
    stalled = speed < cfg["stall_speed_m_s"]
    still = (still if still is not None else t) if stalled else None
    done = still is not None and t - still >= cfg["stall_dwell_s"]
    if not closing and travel < open_g - cfg["open_reached_margin_m"]:
        done = False
    return done, still


class OpenCatch:
    """The open-catch state of one hand. States: "pass" (no hold), "ramp" (ramp-out running, stream held), "opening" (open released,
    waiting for the fingers to stop open)."""

    def __init__(self, model, side, fq, fv, open_g=None):
        self.model, self.side, self.fq, self.fv = model, side, fq, fv
        self.open_g = TASK_CFG.get("finger_open", 0.044) if open_g is None else open_g
        self.state = "pass"
        self.value = None          # the open command kept (its gripper target)
        self.force = None          # the kept command's own max_force (N), None if the command had none (M255)
        self.still = None          # stop-test state of the opening
        self.log = []              # entries {"tick", "kind", ...} for the report

    @staticmethod
    def position(value):
        """The finger position of a gripper command: a plain number, or the {"position": ..} dictionary the close command uses. None if neither."""
        if isinstance(value, dict):
            value = value.get("position")
        return None if value is None else float(value)

    @staticmethod
    def max_force(value):
        """The max_force (N) of a gripper command given as {"position": .., "max_force": ..}; None for a plain number or a dict without it (M255)."""
        if isinstance(value, dict) and value.get("max_force") is not None:
            return float(value["max_force"])
        return None

    def is_open(self, value):
        """True if a gripper command is an open command (its position at or above OPEN_CATCH_CFG["open_command_min_m"])."""
        p = self.position(value)
        return p is not None and p >= OPEN_CATCH_CFG["open_command_min_m"]

    def on_open(self, value, holding, t, tick):
        """An open command for this hand. Returns True if it was caught (or consumed by a hold already running), False if it passes."""
        p = self.position(value)
        if self.state != "pass":
            same = abs(p - self.value) < 1e-9
            self.log.append({"tick": tick, "kind": "repeat_ignored" if same else "different_ignored", "value": p})
            return True
        if not holding:
            # M255 (David, option c): the open passes at once; if held mass is still on, the existing ramp-out to 0 starts now, in this tick.
            if hmr.mass_at(self.model, self.side, t) > 0.0:
                hmr.set_target(self.model, self.side, 0.0, t, tick)
                self.log.append({"tick": tick, "kind": "ramp_no_hold", "value": p})
            return False
        self.state, self.value, self.force = "ramp", p, self.max_force(value)
        hmr.set_target(self.model, self.side, 0.0, t, tick)
        self.log.append({"tick": tick, "kind": "caught", "value": p})
        return True

    def step(self, data, t, tick, holding):
        """Called every tick while a hold is running. Returns what the loop does this tick:
        "hold" (keep the stream held), "release" (send the open command now), "open" (the open is out; keep waiting),
        "resume" (the source may be asked again; the hold is over), or "pass" (no hold running)."""
        if self.state == "pass":
            return "pass"
        if self.state == "ramp":
            if not holding:
                self.state = "pass"
                self.log.append({"tick": tick, "kind": "failed_hold"})
                return "resume"
            if hmr.finished(self.model, self.side, t):
                self.state = "opening"
                self.log.append({"tick": tick, "kind": "released"})
                return "release"
            return "hold"
        travel, speed = float(data.qpos[self.fq]), abs(float(data.qvel[self.fv]))
        done, self.still = stall_done(travel, speed, self.still, t, False, PICK_PLACE_CFG, self.open_g)
        if done:
            self.state = "pass"
            self.log.append({"tick": tick, "kind": "opened"})
            return "resume"
        return "open"


class SourceGate:
    """Decides, each tick, whether a source is asked for commands, which of its commands pass, and when a caught open is released
    (decisions M253). Used by the control loop (src/control_loop.py) for every source, and by the synthetic tests."""

    def __init__(self, chunk_period):
        self.period = chunk_period
        self.chunk_of = {}          # source name -> the chunk being played
        self.paused_by = {}         # source name -> the set of reasons it is held for ("open": an open is caught; "weigh": a grasp is weighed); not asked while non-empty
        self.held_src = {}          # (side, reason) -> the source held for that reason
        self.owner = {}             # side -> the source that last sent a command for that hand (the weigh pauses it)

    def hold_source(self, side, reason):
        """Holds the source that owns `side` for `reason` (M258): it is not asked, and its chunk is discarded. Returns False when no source
        has sent a command for that hand yet (nothing to hold)."""
        name = self.owner.get(side)
        if name is None:
            return False
        self.held_src[(side, reason)] = name
        self.paused_by.setdefault(name, set()).add(reason)
        self.chunk_of.pop(name, None)
        return True

    def release_source(self, side, reason):
        """Ends one hold of `side` for `reason`; the source is asked again once no reason holds it."""
        name = self.held_src.pop((side, reason), None)
        if name is None:
            return
        reasons = self.paused_by.get(name, set())
        reasons.discard(reason)
        if not reasons:
            self.paused_by.pop(name, None)

    def output(self, name, ask, frame):
        """What this source gives this tick. `ask()` asks the source (its step). Returns None while paused, one command of a chunk
        on a due tick, or the source's own output (a command, a list, a chunk, or None)."""
        if self.paused_by.get(name):
            return None
        if name in self.chunk_of and self.chunk_of[name].has_more():
            return self.chunk_of[name].next(frame)
        self.chunk_of.pop(name, None)
        out = ask()
        if is_chunk(out):
            self.chunk_of[name] = ChunkPlayer(out["commands"], frame, self.period)
            return self.chunk_of[name].next(frame)
        return out

    def filter(self, name, cmds, catch, hold, t, frame, log):
        """The commands of one source output that pass. An open command for a hand is caught while that hand holds: the source is
        paused, the rest of its chunk is discarded, and the commands after the open in this output are dropped. Returns the kept commands."""
        kept = []
        for cmd in cmds:
            if isinstance(cmd, dict) and cmd.get("mode") == "arms":
                for s in list(cmd.get("arm_targets", {})) + list(cmd.get("gripper", {})):
                    self.owner[s] = name                      # M258: the source that commands this hand (the weigh pauses it)
            if isinstance(cmd, dict) and cmd.get("mode") == "arms" and cmd.get("gripper"):
                for s, v in cmd["gripper"].items():
                    if catch[s].is_open(v) and catch[s].on_open(v, hold[s].holding, t, frame):
                        self.hold_source(s, "open")
                        log.append({"tick": frame, "side": s, "kind": catch[s].log[-1]["kind"], "source": name})
                        return kept
            kept.append(cmd)
        return kept

    def hold_steps(self, catch, hold, data, t, frame, apply, log):
        """Each tick, for each hand: release the open once the ramp has finished (apply(side, value, force) sends the gripper open, with the
        caught command's own max_force, None if it had none), and resume the source once the gripper has opened."""
        for s in catch:
            st = catch[s].step(data, t, frame, hold[s].holding)
            if st == "release":
                apply(s, catch[s].value, catch[s].force)
                log.append({"tick": frame, "side": s, "kind": "open_released", "value": catch[s].value, "force": catch[s].force})
            elif st == "resume":
                self.release_source(s, "open")
                log.append({"tick": frame, "side": s, "kind": catch[s].log[-1]["kind"]})
