"""On the station table, in the left arm's lane: a cashier's coin till and a customer change tray. The till is a fixed,
open-topped box with three compartments, one per US coin (nickels, quarters, dimes, near to far), two coins lying flat
in each. The change tray is a fixed, open-topped box inboard of the till. The customer is owed 35 cents: the policy has
to put coins worth exactly 35 cents in the tray. Two sets do it with the coins on hand (a quarter and a dime, or a
quarter and two nickels); one coin of each kind is 40 cents, an overpayment, and scores 0.

Coins are counted by where they end up: in the tray (centre over the tray's floor, lowest point below its rim), in the
till (centre over a compartment's floor, lowest point below its rim) or loose (anywhere else, including off the table).

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/make-change/make_change.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "The customer is owed 35 cents. Take exactly 35 cents from the till and put it in the change tray." \\
           --scorer task_score --max-steps 2400
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).

# US Mint specifications: diameter, thickness, mass, value in cents.
COINS = {
    "nickel": dict(d=0.02121, t=0.00195, kg=0.005000, cents=5, color=(0.70, 0.71, 0.72)),
    "quarter": dict(d=0.02426, t=0.00175, kg=0.005670, cents=25, color=(0.76, 0.77, 0.78)),
    "dime": dict(d=0.01791, t=0.00135, kg=0.002268, cents=10, color=(0.78, 0.79, 0.80)),
}
TARGET_CENTS = 35
ORDER = ("nickel", "quarter", "dime")  # compartments near to far along x
PER_KIND = 2

# Till: compartments 50 mm along x (the fingers close along y, so the dividers sit off the fingers' sides) and 180 mm
# along y (the two coins of a kind sit 80 mm apart, with room for an open finger beside each).
TILL_Y = 0.305  # in line with the left arm's base
TILL_FRONT = 0.140  # x of the till's front face
WALL_T = 0.005
WALL_H = 0.015
COMP_X = 0.050
COMP_Y = 0.180
COIN_DY = 0.040  # each coin's centre this far from the till's centre line
COMP_CX = tuple(TILL_FRONT + WALL_T + COMP_X / 2 + k * (COMP_X + WALL_T) for k in range(len(ORDER)))
TILL_LEN = len(ORDER) * COMP_X + (len(ORDER) + 1) * WALL_T
TILL_MID_X = TILL_FRONT + TILL_LEN / 2
DIVIDER_XS = tuple(TILL_FRONT + WALL_T / 2 + k * (COMP_X + WALL_T) for k in range(len(ORDER) + 1))

# Change tray: 100 x 100 mm inside, inboard of the till.
TRAY_XY = (0.225, 0.110)
TRAY_IN = 0.100

TILL_COLOR = (0.18, 0.18, 0.20)
TRAY_COLOR = (0.30, 0.30, 0.34)


def _walls(prefix, cx, cy, inner_x, inner_y, color):
    """Four fixed walls (no mass) around an inner_x by inner_y floor centred at (cx, cy); the floor is the table."""
    ox, oy = inner_x / 2 + WALL_T / 2, inner_y / 2 + WALL_T / 2
    z = TOP + WALL_H / 2
    return [
        Box(f"{prefix}_wall_near", pos=(cx - ox, cy, z), size=(WALL_T, inner_y + 2 * WALL_T, WALL_H), color=color),
        Box(f"{prefix}_wall_far", pos=(cx + ox, cy, z), size=(WALL_T, inner_y + 2 * WALL_T, WALL_H), color=color),
        Box(f"{prefix}_wall_left", pos=(cx, cy + oy, z), size=(inner_x, WALL_T, WALL_H), color=color),
        Box(f"{prefix}_wall_right", pos=(cx, cy - oy, z), size=(inner_x, WALL_T, WALL_H), color=color),
    ]


TILL = _walls("till", TILL_MID_X, TILL_Y, TILL_LEN - 2 * WALL_T, COMP_Y, TILL_COLOR) + [
    Box(f"till_divider_{k}", pos=(DIVIDER_XS[k], TILL_Y, TOP + WALL_H / 2), size=(WALL_T, COMP_Y, WALL_H), color=TILL_COLOR)
    for k in range(1, len(ORDER))
]
TRAY = _walls("tray", *TRAY_XY, TRAY_IN, TRAY_IN, TRAY_COLOR)

STARTS = {
    f"coin_{kind}_{n}": (COMP_CX[k], TILL_Y + (COIN_DY if n == 1 else -COIN_DY))
    for k, kind in enumerate(ORDER)
    for n in (1, 2)
}  # coin_<kind>_1 on the outboard (left) side, coin_<kind>_2 inboard


def kind_of(name) -> str:
    return name.split("_")[1]


COIN_OBJECTS = [
    Cylinder(name, pos=(x, y, TOP + COINS[kind_of(name)]["t"] / 2 + 0.0005), radius=COINS[kind_of(name)]["d"] / 2,
             height=COINS[kind_of(name)]["t"], mass=COINS[kind_of(name)]["kg"], color=COINS[kind_of(name)]["color"])
    for name, (x, y) in STARTS.items()
]
OBJECTS = [*COIN_OBJECTS, *TILL, *TRAY]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

EPS = 1e-4
EXACT_WITH_LOOSE = 0.75  # exact change in the tray, but a coin left loose on the counter or the floor
UNDER_WEIGHT = 0.5  # under the target: 0.5 x cents in the tray / 35


# A coin's lowest point comes from its centre and axis, not from ``low``: Arena builds ``low``/``high`` from the corners
# of the cylinder's local box, so a tilted coin spinning on its own axis would read lower than it is (as in pin-fit).
def _axis_z(quat_xyzw) -> float:
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return 1.0 - 2.0 * (x * x + y * y)


def _lowest(state, kind) -> float:
    """World z of the coin's lowest point; exact for a cylinder at any tilt."""
    c = min(1.0, abs(_axis_z(state.quat_xyzw)))
    spec = COINS[kind]
    return float(state.pos[2]) - (spec["t"] / 2 * c + spec["d"] / 2 * math.sqrt(max(0.0, 1.0 - c * c)))


def _inside(state, kind, cx, cy, inner_x, inner_y) -> bool:
    """Centre over the inner floor (0.1 mm epsilon) and lowest point below the rim."""
    x, y = float(state.pos[0]), float(state.pos[1])
    over = abs(x - cx) <= inner_x / 2 + EPS and abs(y - cy) <= inner_y / 2 + EPS
    return over and _lowest(state, kind) < TOP + WALL_H


def where(state, kind) -> str:
    """'tray', 'till' or 'loose'."""
    if _inside(state, kind, *TRAY_XY, TRAY_IN, TRAY_IN):
        return "tray"
    if any(_inside(state, kind, cx, TILL_Y, COMP_X, COMP_Y) for cx in COMP_CX):
        return "till"
    return "loose"


def _tally(objects):
    places = {name: where(objects[name], kind_of(name)) for name in STARTS}
    cents = sum(COINS[kind_of(n)]["cents"] for n, p in places.items() if p == "tray")
    loose = sum(1 for p in places.values() if p == "loose")
    return cents, loose


def success(objects) -> bool:
    """Exactly 35 cents in the tray, no coin loose, every coin at rest; holds for 1 s."""
    cents, loose = _tally(objects)
    return cents == TARGET_CENTS and loose == 0 and all(objects[n].resting() for n in STARTS)


def distance(objects) -> float:
    """Cents still to go (negative past the target), in dollars, as a progress signal."""
    cents, _ = _tally(objects)
    return (TARGET_CENTS - cents) / 100.0


def score(objects) -> float:
    """Over 35 cents in the tray: 0. Exactly 35 and no coin loose: 1.0. Exactly 35 with a coin loose: 0.75.
    Under 35: 0.5 x cents in the tray / 35 (0 for an empty tray, 0.43 at 30 cents)."""
    cents, loose = _tally(objects)
    if cents > TARGET_CENTS:
        return 0.0
    if cents == TARGET_CENTS:
        return 1.0 if loose == 0 else EXACT_WITH_LOOSE
    return UNDER_WEIGHT * cents / TARGET_CENTS


# --- Scripted demo: a quarter and a dime, left arm only, yaw 0 (the fingers close along y, across the coin). ---
# UNVERIFIED on the simulator: a flat coin is 1.35-1.95 mm thick, so the pads must close with their tips within a
# fraction of a millimetre of the table. PAD_CLEAR and the pre-shaped opening are first guesses to tune on the stack.
TRAVEL = TOP + 0.100
PAD_CLEAR = 0.0004  # pad tips this far above the table when they close on a coin's rim
PICK_Z = TOP + PAD_CLEAR + PAD_TIP_PAST_TCP
RELEASE_Z = TOP + WALL_H + 0.003 + PAD_TIP_PAST_TCP  # pad tips 3 mm above the tray's rim; the coin drops about 17 mm
PRESHAPE = 0.010  # open only this much wider than the coin before descending
DROP_SPOTS = ((TRAY_XY[0] - 0.025, TRAY_XY[1]), (TRAY_XY[0] + 0.025, TRAY_XY[1]))


def _coin_to_tray(name, spot):
    kind = kind_of(name)
    px, py = STARTS[name]
    qx, qy = spot
    preshape = -(COINS[kind]["d"] + PRESHAPE) / 2  # finger target = minus half the gap, as in fragile-stow
    return [
        Waypoint(1.5, (px, py, TRAVEL), preshape, f"over the {kind}"),
        Waypoint(1.5, (px, py, PICK_Z), preshape, "descend to the table"),
        Waypoint(0.6, (px, py, PICK_Z), preshape, "settle"),
        Waypoint(1.5, (px, py, PICK_Z), FINGER_CLOSED, "close on the rim"),
        Waypoint(1.2, (px, py, TRAVEL), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (qx, qy, TRAVEL), FINGER_CLOSED, "carry to the tray"),
        Waypoint(0.6, (qx, qy, TRAVEL), FINGER_CLOSED, "settle over the tray"),
        Waypoint(1.2, (qx, qy, RELEASE_Z), FINGER_CLOSED, "lower to just above the rim"),
        Waypoint(0.8, (qx, qy, RELEASE_Z), FINGER_CLOSED, "settle"),
        Waypoint(1.0, (qx, qy, RELEASE_Z), FINGER_OPEN, "release"),
        Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, "retreat"),
    ]


def demo():
    """The inboard quarter, then the inboard dime, each dropped into the tray: 25.8 s of plan, then the 1 s hold."""
    return {"left": _coin_to_tray("coin_quarter_2", DROP_SPOTS[0]) + _coin_to_tray("coin_dime_2", DROP_SPOTS[1])}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat_tilt(roll=0.0):
        return np.array([math.sin(roll / 2), 0.0, 0.0, math.cos(roll / 2)])

    def coin(name, x, y, bottom=TOP, roll=0.0, vel=(0.0, 0.0, 0.0)):
        spec = COINS[kind_of(name)]
        c, s = abs(math.cos(roll)), abs(math.sin(roll))
        z = bottom + spec["t"] / 2 * c + spec["d"] / 2 * s
        pos = np.array([x, y, z])
        half = np.array([spec["d"] / 2, spec["d"] / 2, spec["t"] / 2])
        return ObjectState(pos, quat_tilt(roll), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {n: coin(n, *xy) for n, xy in STARTS.items()}
    tx, ty = TRAY_XY

    def scene(**moved):
        return {**start, **{f"coin_{k}": st for k, st in moved.items()}}

    def tray(name, dx=0.0, dy=0.0, **kw):
        return coin(name, tx + dx, ty + dy, **kw)

    half_in = TRAY_IN / 2
    cases = {
        "start": (scene(), 0.0, False),
        "quarter + dime": (scene(quarter_2=tray("coin_quarter_2", -0.025), dime_2=tray("coin_dime_2", 0.025)), 1.0, True),
        "quarter + two nickels": (scene(quarter_1=tray("coin_quarter_1", -0.03), nickel_1=tray("coin_nickel_1", 0.0), nickel_2=tray("coin_nickel_2", 0.03)), 1.0, True),
        "one of each (40c, overpaid)": (scene(quarter_2=tray("coin_quarter_2", -0.03), dime_2=tray("coin_dime_2"), nickel_2=tray("coin_nickel_2", 0.03)), 0.0, False),
        "everything in the tray (80c)": ({n: tray(n, 0.012 * (k - 3)) for k, n in enumerate(STARTS)}, 0.0, False),
        "two quarters (50c)": (scene(quarter_1=tray("coin_quarter_1", -0.02), quarter_2=tray("coin_quarter_2", 0.02)), 0.0, False),
        "quarter only (25c)": (scene(quarter_2=tray("coin_quarter_2")), 0.5 * 25 / 35, False),
        "two dimes + a nickel (25c)": (scene(dime_1=tray("coin_dime_1", -0.03), dime_2=tray("coin_dime_2"), nickel_1=tray("coin_nickel_1", 0.03)), 0.5 * 25 / 35, False),
        "quarter + nickel (30c)": (scene(quarter_2=tray("coin_quarter_2", -0.02), nickel_2=tray("coin_nickel_2", 0.02)), 0.5 * 30 / 35, False),
        "a dime dropped on the counter, nothing in the tray": (scene(dime_2=coin("coin_dime_2", 0.30, 0.10)), 0.0, False),
        "35c in the tray, a nickel dropped on the counter": (scene(quarter_2=tray("coin_quarter_2", -0.025), dime_2=tray("coin_dime_2", 0.025), nickel_1=coin("coin_nickel_1", 0.30, 0.10)), 0.75, False),
        "35c in the tray, a nickel off the table": (scene(quarter_2=tray("coin_quarter_2", -0.025), dime_2=tray("coin_dime_2", 0.025), nickel_1=coin("coin_nickel_1", 0.30, 0.10, bottom=0.0)), 0.75, False),
        "35c, the dime upside down": (scene(quarter_2=tray("coin_quarter_2", -0.025), dime_2=tray("coin_dime_2", 0.025, roll=math.pi)), 1.0, True),
        "35c, the dime on top of the quarter": (scene(quarter_2=tray("coin_quarter_2"), dime_2=tray("coin_dime_2", bottom=TOP + COINS["quarter"]["t"])), 1.0, True),
        "35c, the dime on edge leaning on the tray wall": (scene(quarter_2=tray("coin_quarter_2"), dime_2=tray("coin_dime_2", half_in - 0.006, roll=math.radians(70))), 1.0, True),
        "35c but the dime still sliding": (scene(quarter_2=tray("coin_quarter_2"), dime_2=tray("coin_dime_2", 0.02, vel=(0.05, 0, 0))), 1.0, False),
        "dime held 30 mm up over the tray (not counted)": (scene(quarter_2=tray("coin_quarter_2"), dime_2=tray("coin_dime_2", bottom=TOP + 0.030)), 0.5 * 25 / 35, False),
        "dime lying on the tray rim (loose)": (scene(quarter_2=tray("coin_quarter_2"), dime_2=coin("coin_dime_2", tx, ty + half_in + WALL_T / 2, bottom=TOP + WALL_H)), 0.5 * 25 / 35, False),
        "dime centre just outside the tray floor (loose)": (scene(quarter_2=tray("coin_quarter_2"), dime_2=tray("coin_dime_2", half_in + 0.001)), 0.5 * 25 / 35, False),
        "a nickel moved to the dime compartment (still in the till)": (scene(quarter_2=tray("coin_quarter_2", -0.025), dime_2=tray("coin_dime_2", 0.025), nickel_1=coin("coin_nickel_1", COMP_CX[2], TILL_Y)), 1.0, True),
        "a nickel lying on a till divider (loose)": (scene(quarter_2=tray("coin_quarter_2", -0.025), dime_2=tray("coin_dime_2", 0.025), nickel_1=coin("coin_nickel_1", DIVIDER_XS[1], TILL_Y, bottom=TOP + WALL_H)), 0.75, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)

    # Every start position: coins inside their compartments, clear of the walls, not touching each other.
    for name, (x, y) in STARTS.items():
        assert where(start[name], kind_of(name)) == "till", name
    # The demo drop spots: the quarter and dime fit side by side on the tray floor.
    d = math.dist(*DROP_SPOTS)
    assert d > (COINS["quarter"]["d"] + COINS["dime"]["d"]) / 2, d
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the left arm's plan replayed through the
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
