"""The gap rule (M294, SF-29): what a pair's gap must do in the next tick. Numbers in, numbers out.

Modes (config GAP_RULE["mode"]):
  "ours"         gap above the margin: it may close at most at approach_per_s x (gap - margin) (m/s).
                 gap below the margin: it must grow at approach_per_s x (margin - gap), capped at max_grow_speed_m_s.
                 Inside the slowing band (margin to margin + 1 cm, released at 2 cm) only the closing speed is limited:
                 the rule above gives no push there (the gap is above the margin).
  "paper_plain"  no gap rule: no speed limit above the margin; a pair below its margin is pushed back to the margin
                 in one step. (A stand-in for the paper's plain behaviour; Kong's compiled filter is not readable, SF-29.)

Margins and the band come from the touch rule of the pair (Rule), not from here (SF-29).
"""

from src.safety_filter import config as cfg

MODES = ("ours", "paper_plain")


def mode():
    """The gap rule mode now in the config. Raises ValueError for an unknown value."""
    m = cfg.GAP_RULE["mode"]
    if m not in MODES:
        raise ValueError(f"GAP_RULE['mode'] must be one of {MODES}, not {m!r}")
    return m


def allowed_approach_speed(gap, margin):
    """Gap above the margin: the fastest the gap may close, m/s ("ours" mode): approach_per_s x (gap - margin).
    Zero at the margin. Returns 0.0 if the gap is below the margin (there it must grow instead)."""
    return max(0.0, cfg.GAP_RULE["approach_per_s"] * (gap - margin))


def required_grow_speed(gap, margin):
    """Gap below the margin: the slowest the gap may grow, m/s ("ours" mode): approach_per_s x (margin - gap),
    capped at max_grow_speed_m_s. Returns 0.0 if the gap is at or above the margin."""
    return min(cfg.GAP_RULE["max_grow_speed_m_s"], max(0.0, cfg.GAP_RULE["approach_per_s"] * (margin - gap)))


def required_gap_next(gap_last, margin, dt):
    """The smallest gap the pair may have after the next tick, given its gap at the last filtered command.

    Args: gap_last (m): gap at the last filtered command. margin (m): the pair's margin. dt (s): tick length.
    Returns: float (m). "ours": gap_last - allowed_speed x dt above the margin; gap_last + grow_speed x dt below it.
    "paper_plain": the margin itself (the pair must not end below its margin; above it nothing limits the approach
    except that limit). The caller measures the shortfall as required - actual gap.
    """
    if mode() == "paper_plain":
        return margin
    if gap_last >= margin:
        return gap_last - allowed_approach_speed(gap_last, margin) * dt
    return gap_last + required_grow_speed(gap_last, margin) * dt


def shortfall(gap_last, gap_now, margin, dt):
    """How far short of the rule the pair is at the new pose: max(0, required gap - new gap), metres."""
    return max(0.0, required_gap_next(gap_last, margin, dt) - gap_now)
