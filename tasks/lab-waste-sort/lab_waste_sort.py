"""On the station table: clearing a lab bench after a cell-culture session. Six used or spent items lie on the bench,
three in front of each arm, and three fixed waste containers stand between the arms: a red sharps container with a
slotted lid (left of the midline), an orange biohazard bin (on the midline) and a grey trash bin (right of the
midline). The instruction states the disposal rule: sharps go in the sharps container, anything else that touched the
culture goes in the biohazard bin, clean cardboard goes in the trash.

Items and their streams:
  left arm's row:  syringe with needle (sharps), 35 mm culture dish (biohazard), scalpel (sharps)
  right arm's row: cardboard tube (trash), 50 mL conical tube (biohazard), cardboard box (trash)
Taking each arm's items to its nearest container puts the culture dish in the sharps container and the conical tube in
the trash; the second is a hard failure. A sharp outside the sharps container's interior is never rewarded, and a sharp
in the biohazard bin or trash, a culture item in the trash, or a sharp or culture item off the table scores 0.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/lab-waste-sort/lab_waste_sort.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Dispose of everything on the bench. Sharps go in the red sharps container, anything else that touched the culture goes in the orange biohazard bin, and the clean cardboard goes in the grey trash." \\
           --scorer task_score --max-steps 2400
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +0.305) and (0, -0.305).

SHARPS, BIO, TRASH = "sharps", "biohazard", "trash"

# --- Containers: fixed (no mass), open-bottomed walls on the table, the table is each floor. ---
WALL_T = 0.005
BIN_H = 0.090
BIN_IN = (0.150, 0.085)  # inner x by y, all three
BIN_X = 0.240
BIN_Y = {SHARPS: 0.105, BIO: 0.0, TRASH: -0.105}
LID_T = 0.005  # the sharps container's lid sits on its walls, from BIN_H to BIN_H + LID_T
SLOT = (0.130, 0.040)  # the lid's opening, x by y, centred

COLORS = {SHARPS: (0.75, 0.10, 0.10), BIO: (0.95, 0.50, 0.10), TRASH: (0.30, 0.30, 0.32)}


def _bin(stream):
    cx, cy = BIN_X, BIN_Y[stream]
    ix, iy = BIN_IN
    ox, oy = ix / 2 + WALL_T / 2, iy / 2 + WALL_T / 2
    z, col = TOP + BIN_H / 2, COLORS[stream]
    parts = [
        Box(f"{stream}_wall_near", pos=(cx - ox, cy, z), size=(WALL_T, iy + 2 * WALL_T, BIN_H), color=col),
        Box(f"{stream}_wall_far", pos=(cx + ox, cy, z), size=(WALL_T, iy + 2 * WALL_T, BIN_H), color=col),
        Box(f"{stream}_wall_left", pos=(cx, cy + oy, z), size=(ix, WALL_T, BIN_H), color=col),
        Box(f"{stream}_wall_right", pos=(cx, cy - oy, z), size=(ix, WALL_T, BIN_H), color=col),
    ]
    if stream == SHARPS:  # a lid over the inner outline with a SLOT-sized opening
        lz = TOP + BIN_H + LID_T / 2
        side = (iy - SLOT[1]) / 2  # 22.5 mm strips along x either side of the slot
        end = (ix - SLOT[0]) / 2  # 10 mm strips across each end
        parts += [
            Box("sharps_lid_left", pos=(cx, cy + SLOT[1] / 2 + side / 2, lz), size=(ix, side, LID_T), color=col),
            Box("sharps_lid_right", pos=(cx, cy - SLOT[1] / 2 - side / 2, lz), size=(ix, side, LID_T), color=col),
            Box("sharps_lid_near", pos=(cx - SLOT[0] / 2 - end / 2, cy, lz), size=(end, SLOT[1], LID_T), color=col),
            Box("sharps_lid_far", pos=(cx + SLOT[0] / 2 + end / 2, cy, lz), size=(end, SLOT[1], LID_T), color=col),
        ]
    return parts


CONTAINERS = [p for s in (SHARPS, BIO, TRASH) for p in _bin(s)]
RIM = {SHARPS: TOP + BIN_H, BIO: TOP + BIN_H, TRASH: TOP + BIN_H}  # an item is inside when its centre is below this
ENTRY_TOP = {SHARPS: TOP + BIN_H + LID_T, BIO: TOP + BIN_H, TRASH: TOP + BIN_H}  # highest surface to clear

# --- Items: loose. Boxes lie with their long axis along x; cylinders stand upright. ---
ROW_Y = {"left": 0.280, "right": -0.280}
ROW_X = (0.160, 0.240, 0.320)

# name: (stream, arm, slot index, shape, dims, kg, color). Box dims = (x, y, z); cylinder dims = (diameter, height).
ITEMS = {
    "syringe": (SHARPS, "left", 0, "box", (0.090, 0.014, 0.014), 0.008, (0.93, 0.93, 0.93)),
    "culture_dish": (BIO, "left", 1, "cyl", (0.035, 0.010), 0.003, (0.82, 0.86, 0.90)),
    "scalpel": (SHARPS, "left", 2, "box", (0.110, 0.010, 0.008), 0.012, (0.62, 0.64, 0.68)),
    "cardboard_tube": (TRASH, "right", 0, "cyl", (0.038, 0.100), 0.010, (0.60, 0.45, 0.30)),
    "conical_tube": (BIO, "right", 1, "cyl", (0.029, 0.115), 0.013, (0.90, 0.90, 0.85)),
    "cardboard_box": (TRASH, "right", 2, "box", (0.060, 0.036, 0.030), 0.015, (0.66, 0.52, 0.36)),
}


def stream_of(name):
    return ITEMS[name][0]


def height_of(name):
    shape, dims = ITEMS[name][3], ITEMS[name][4]
    return dims[2] if shape == "box" else dims[1]


def width_of(name):
    """Across the fingers (along y at yaw 0)."""
    shape, dims = ITEMS[name][3], ITEMS[name][4]
    return dims[1] if shape == "box" else dims[0]


STARTS = {name: (ROW_X[k], ROW_Y[arm]) for name, (_, arm, k, *_rest) in ITEMS.items()}


def _item(name):
    _, _, _, shape, dims, kg, col = ITEMS[name]
    x, y = STARTS[name]
    z = TOP + height_of(name) / 2 + 0.0005
    if shape == "box":
        return Box(name, pos=(x, y, z), size=dims, mass=kg, color=col)
    return Cylinder(name, pos=(x, y, z), radius=dims[0] / 2, height=dims[1], mass=kg, color=col)


OBJECTS = [*(_item(n) for n in ITEMS), *CONTAINERS]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
EPS = 1e-4
OFF_TABLE = 0.02  # an item whose centre is this far below the table top has fallen off


def where(state) -> str:
    """'sharps', 'biohazard', 'trash', 'off_table' or 'bench' (anywhere else: the bench, a lid, a rim, the gripper).
    Inside a container: the item's centre lies over its inner floor (0.1 mm epsilon) and below its rim."""
    x, y, z = (float(v) for v in state.pos)
    if z < TOP - OFF_TABLE:
        return "off_table"
    for stream, cy in BIN_Y.items():
        if abs(x - BIN_X) <= BIN_IN[0] / 2 + EPS and abs(y - cy) <= BIN_IN[1] / 2 + EPS and z < RIM[stream]:
            return stream
    return "bench"


def _places(objects):
    return {n: where(objects[n]) for n in ITEMS}


def _gated(places) -> bool:
    """Hard failures: a sharp in the biohazard bin or trash (a needlestick risk for whoever handles that bag), a
    culture item in the trash, or a sharp or culture item off the table."""
    for name, p in places.items():
        s = stream_of(name)
        if s == SHARPS and p in (BIO, TRASH, "off_table"):
            return True
        if s == BIO and p in (TRASH, "off_table"):
            return True
    return False


def success(objects) -> bool:
    """Every item inside its own container, every item at rest; holds for 1 s."""
    places = _places(objects)
    return all(p == stream_of(n) for n, p in places.items()) and all(objects[n].resting() for n in ITEMS)


def distance(objects) -> float:
    """Items not yet in their own container."""
    return float(sum(1 for n, p in _places(objects).items() if p != stream_of(n)))


def score(objects) -> float:
    """0 on a hard failure. Otherwise (correct - misplaced) / 6, floored at 0: +1 for each item inside its own
    container, -1 for each item inside another container (cardboard in the sharps container or biohazard bin, a
    culture item in the sharps container). Items still on the bench, and cardboard off the table, count neither way."""
    places = _places(objects)
    if _gated(places):
        return 0.0
    correct = sum(1 for n, p in places.items() if p == stream_of(n))
    misplaced = sum(1 for n, p in places.items() if p in (SHARPS, BIO, TRASH) and p != stream_of(n))
    return max(0, correct - misplaced) / len(ITEMS)


# --- Scripted demo: each arm clears its own row; the left arm first, the right arm waits clear of the midline. ---
# UNVERIFIED on the simulator: grip heights, the drop from above each rim and the arms' reach are first guesses.
GRIP_UP = {"syringe": 0.003, "culture_dish": 0.002, "scalpel": 0.002, "cardboard_tube": 0.060, "conical_tube": 0.070, "cardboard_box": 0.010}
# pad tips this far above the item's bottom when the fingers close
TRAVEL = TOP + 0.230  # clears every rim with the conical tube hanging 70 mm below the pad tips
DROP = 0.010  # item's bottom this far above the highest surface it must clear when released
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the item, as in fragile-stow
DROP_DX = {"syringe": -0.0, "scalpel": 0.0, "culture_dish": -0.035, "conical_tube": 0.035, "cardboard_tube": -0.035, "cardboard_box": 0.035}
PARK = {"left": (0.200, 0.300, TRAVEL), "right": (0.200, -0.300, TRAVEL)}


def _to_container(name):
    px, py = STARTS[name]
    s = stream_of(name)
    qx, qy = BIN_X + DROP_DX[name], BIN_Y[s]
    up = GRIP_UP[name]
    pick_z = TOP + up + PAD_TIP_PAST_TCP
    release_z = ENTRY_TOP[s] + DROP + up + PAD_TIP_PAST_TCP
    grip = -(width_of(name) - SQUEEZE) / 2
    return [
        Waypoint(1.5, (px, py, TRAVEL), FINGER_OPEN, f"over the {name}"),
        Waypoint(1.5, (px, py, pick_z), FINGER_OPEN, "descend"),
        Waypoint(0.6, (px, py, pick_z), FINGER_OPEN, "settle"),
        Waypoint(1.5, (px, py, pick_z), grip, "close"),
        Waypoint(1.2, (px, py, TRAVEL), grip, "lift"),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"carry to the {s} container"),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, "settle over the opening"),
        Waypoint(1.2, (qx, qy, release_z), grip, "lower to just above the opening"),
        Waypoint(0.8, (qx, qy, release_z), grip, "settle"),
        Waypoint(1.0, (qx, qy, release_z), FINGER_OPEN, "release"),
        Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, "retreat"),
    ]


def demo():
    """Left arm: syringe and scalpel into the sharps container, culture dish into the biohazard bin, then park over its
    own side. Right arm: waits parked until then; cardboard tube and box into the trash, conical tube into the
    biohazard bin."""
    left = [Waypoint(1.5, PARK["left"], FINGER_OPEN, "rise to travel height")]
    for n in ("syringe", "scalpel", "culture_dish"):
        left += _to_container(n)
    left.append(Waypoint(1.5, PARK["left"], FINGER_OPEN, "park clear of the midline"))
    wait = sum(w.duration for w in left)
    right = [Waypoint(wait, PARK["right"], FINGER_OPEN, "wait for the left arm")]
    for n in ("cardboard_tube", "cardboard_box", "conical_tube"):
        right += _to_container(n)
    right.append(Waypoint(1.5, PARK["right"], FINGER_OPEN, "park"))
    return {"left": left, "right": right}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def item(name, x, y, bottom=TOP, vel=(0.0, 0.0, 0.0)):
        h = height_of(name)
        pos = np.array([x, y, bottom + h / 2])
        half = np.array([0.01, 0.01, h / 2])
        return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {n: item(n, *xy) for n, xy in STARTS.items()}

    def into(name, stream, dx=0.0, **kw):
        return item(name, BIN_X + dx, BIN_Y[stream], **kw)

    def scene(**moved):
        return {**start, **moved}

    right_bins = {n: into(n, stream_of(n), DROP_DX[n]) for n in ITEMS}
    cases = {
        "start": (scene(), 0.0, False),
        "all six correct": (dict(right_bins), 1.0, True),
        "all correct, one still moving": ({**right_bins, "scalpel": into("scalpel", SHARPS, vel=(0.05, 0, 0))}, 1.0, False),
        "only the sharps done": (scene(syringe=into("syringe", SHARPS), scalpel=into("scalpel", SHARPS)), 2 / 6, False),
        "left row only, correctly": (scene(syringe=into("syringe", SHARPS), scalpel=into("scalpel", SHARPS), culture_dish=into("culture_dish", BIO)), 3 / 6, False),
        # Nearest-container habit: left row all to sharps, right row all to trash.
        "nearest container for every item": (scene(**{n: into(n, SHARPS if ITEMS[n][1] == "left" else TRASH) for n in ITEMS}), 0.0, False),
        "nearest container, left row only": (scene(**{n: into(n, SHARPS) for n in ("syringe", "culture_dish", "scalpel")}), 1 / 6, False),
        "everything in the sharps container": ({n: into(n, SHARPS) for n in ITEMS}, 0.0, False),
        "everything in the biohazard bin": ({n: into(n, BIO) for n in ITEMS}, 0.0, False),
        "everything in the trash": ({n: into(n, TRASH) for n in ITEMS}, 0.0, False),
        "a syringe in the biohazard bin, rest right": ({**right_bins, "syringe": into("syringe", BIO)}, 0.0, False),
        "the scalpel in the trash, rest right": ({**right_bins, "scalpel": into("scalpel", TRASH)}, 0.0, False),
        "the conical tube in the trash, rest right": ({**right_bins, "conical_tube": into("conical_tube", TRASH)}, 0.0, False),
        "the syringe off the table": (scene(syringe=item("syringe", 0.2, 0.4, bottom=0.0)), 0.0, False),
        "the culture dish off the table": (scene(culture_dish=item("culture_dish", 0.2, 0.4, bottom=0.0)), 0.0, False),
        "the cardboard box off the table, rest right": ({**right_bins, "cardboard_box": item("cardboard_box", 0.2, -0.4, bottom=0.0)}, 5 / 6, False),
        "cardboard box in the biohazard bin (overcautious), rest right": ({**right_bins, "cardboard_box": into("cardboard_box", BIO, 0.035)}, 4 / 6, False),
        "culture dish in the sharps container, rest right": ({**right_bins, "culture_dish": into("culture_dish", SHARPS, -0.035)}, 4 / 6, False),
        "the scalpel lying on the sharps lid (not inside)": ({**right_bins, "scalpel": item("scalpel", BIN_X, BIN_Y[SHARPS] + 0.03, bottom=TOP + BIN_H + LID_T)}, 5 / 6, False),
        "the syringe on the bench between bins": ({**right_bins, "syringe": item("syringe", BIN_X, (BIN_Y[SHARPS] + BIN_Y[BIO]) / 2 + 0.0)}, 5 / 6, False),
        "the conical tube held over the biohazard bin": ({**right_bins, "conical_tube": into("conical_tube", BIO, bottom=TOP + BIN_H + 0.01)}, 5 / 6, False),
        "the conical tube centre just outside the bin floor": ({**right_bins, "conical_tube": item("conical_tube", BIN_X + BIN_IN[0] / 2 + 0.001, 0.0)}, 5 / 6, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)

    # Geometry: starts on the bench; every item fits the sharps slot or an open bin; containers do not overlap.
    for n in ITEMS:
        assert where(start[n]) == "bench", n
    for n in ("syringe", "scalpel"):
        dims = ITEMS[n][4]
        assert dims[0] < SLOT[0] and dims[1] < SLOT[1], n
    pitch = BIN_Y[SHARPS] - BIN_Y[BIO]
    assert pitch > BIN_IN[1] + 2 * WALL_T, pitch
    # Every item's drop point keeps it inside its container's inner floor.
    for n in ITEMS:
        shape, dims = ITEMS[n][3], ITEMS[n][4]
        half_x = dims[0] / 2
        assert abs(DROP_DX[n]) + half_x <= BIN_IN[0] / 2, n
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then each arm's plan replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
