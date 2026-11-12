"""Link categories of the resolver (SF-28): which move handles a contact on which part of an arm.

Every capsule of an arm gets exactly one category:
  shoulder   links 1 and 2 (and the shoulder mount, link 0, fixed to the torso): handled by step shortening only.
  upper_arm  links 3 and 4: handled by the elbow swing.
  forearm    links 5 and 6: handled by the elbow swing.
  hand       link 7, both palm capsules, every finger capsule, and the held object: handled by the hand slide.
A contact point that an elbow swing cannot move (a point on the straight line from the shoulder to the wrist)
is handled by step shortening instead; the resolver tests that (it needs the pose).

This differs from the segment map stored in robot_shapes.json (SF-22: links 1 to 3 upper arm, 4 to 6 forearm). The
resolver does not read that field; the table below is the only source of the categories (SF-28).

No simulator, no world data.
"""

SHOULDER, UPPER_ARM, FOREARM, HAND = "shoulder", "upper_arm", "forearm", "hand"
CATEGORIES = (SHOULDER, UPPER_ARM, FOREARM, HAND)

# part name of a placed shape -> category (SF-28). The part names are those of robot_shapes.json (and "held" for a held object).
PART_CATEGORY = {
    "shoulder_mount": SHOULDER,   # link 0, fixed to the torso (never moved by any arm joint)
    "link1": SHOULDER, "link2": SHOULDER,
    "link3": UPPER_ARM, "link4": UPPER_ARM,
    "link5": FOREARM, "link6": FOREARM,
    "link7": HAND, "palm": HAND, "finger": HAND, "held": HAND,
    # the three capsules of Kong's compiled filter, rebuilt for the comparison (scripts/safety_filter_kong_comparison.py)
    "kong_upper": UPPER_ARM, "kong_lower": FOREARM, "kong_hand": HAND,
}


def category_of_part(part):
    """The category of an arm part name. Returns (category, classified): an unknown part is treated as shoulder
    (step shortening, the safe fallback) with classified False, so the caller can list it as not built."""
    if part in PART_CATEGORY:
        return PART_CATEGORY[part], True
    return SHOULDER, False


def category_of(shape):
    """The category of a placed shape of an arm (a PlacedShape with .part). See category_of_part."""
    return category_of_part(shape.part)[0]


def category_table(shape_specs):
    """(shape id, arm, part, category, classified) for every capsule of an arm in a list of shape specs
    (the 'shapes' list of robot_shapes.json)."""
    rows = []
    for sp in shape_specs:
        if sp["kind"] == "capsule" and sp["arm"] in ("left", "right"):
            cat, ok = category_of_part(sp["part"])
            rows.append((sp["id"], sp["arm"], sp["part"], cat, ok))
    return rows
