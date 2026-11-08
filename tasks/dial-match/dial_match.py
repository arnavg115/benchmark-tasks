"""On the station table, in front of the left arm: two dials side by side, each a round grey plate with tick marks
every 45 degrees over a 270 degree range. The left one has a blue bar knob on a hinge, with a red tip marking the end
that points; it starts against its 0 degree stop. The right one is a fixed reference whose black pointer (also red
tipped) is set 115 degrees round from 0, between the 90 and 135 degree ticks. The policy has to read the reference and turn the knob to match it.

Both dials share one orientation, turned -57.5 degrees about the vertical: the knob starts pointing 57.5 degrees right
of straight ahead and the reference points 57.5 degrees left of it, so a centred grip turns the wrist from -57.5 to
+57.5 degrees. The knob's end-for-end reading of the target (295 degrees) is past its 270 degree stop, so it can't be
set pointing backwards. The target is 20 degrees from the nearest tick, so snapping to a tick earns at most 0.25.

The score comes from the knob's joint angle: 1.0 within 5 degrees of the reference, 0.5 within 15, 0.25 within 30,
otherwise 0. The reference can't be turned (it is fixed), and the knob can't leave its hinge.

The dial is an articulation loaded from assets/dial.usda (written by make_dial_usda.py) through scenekit.py, copied
unchanged from tasks/lock-before-loading.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/dial-match/dial_match.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Turn the blue knob so it points the same way as the black pointer on the other dial." \\
           --scorer task_score --max-steps 1200
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scenekit import UsdBody  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

import geometry as g  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
DIAL_YAW_DEG = -57.5  # both dials' 0 degree direction, about the vertical from +x
TARGET_DEG = 115.0  # the reference pointer, measured from 0 like the knob
KNOB_XY = (0.250, 0.300)
REF_XY = (0.250, 0.140)  # 160 mm to the right: 70 mm of table between the plates

DIAL = UsdBody("dial", usd="assets/dial.usda", pos=(*KNOB_XY, TOP), rot_xyzw=g.yaw_quat_xyzw(DIAL_YAW_DEG),
               size=(2 * g.PLATE_R, 2 * g.PLATE_R, g.PLATE_H + g.BAR_GAP + g.BAR[2]), articulation=True,
               joint_pos=((g.JOINT, 0.0),))


def _reference():
    """The fixed reference dial: plate, ticks, pointer bar and red tip, all without mass."""
    rx, ry = REF_XY
    parts = [Cylinder("ref_plate", pos=(rx, ry, TOP + g.PLATE_H / 2), radius=g.PLATE_R, height=g.PLATE_H, color=g.PLATE_COLOR)]
    for deg in g.TICK_DEG:
        a = math.radians(DIAL_YAW_DEG + deg)
        parts.append(Box(f"ref_tick_{deg:03d}", pos=(rx + g.TICK_R * math.cos(a), ry + g.TICK_R * math.sin(a), TOP + g.PLATE_H + g.TICK[2] / 2),
                         size=g.TICK, color=g.TICK_COLOR, yaw=a))
    a = math.radians(DIAL_YAW_DEG + TARGET_DEG)
    parts.append(Box("ref_pointer", pos=(rx, ry, TOP + g.BAR_CZ), size=g.BAR, color=g.REF_COLOR, yaw=a))
    parts.append(Box("ref_tip", pos=(rx + g.TIP_CX * math.cos(a), ry + g.TIP_CX * math.sin(a), TOP + g.TIP_CZ), size=g.TIP,
                     color=g.TIP_COLOR, yaw=a))
    return parts


REFERENCE = _reference()
OBJECTS = [DIAL, *REFERENCE]

FULL_DEG, HALF_DEG, QUARTER_DEG = 5.0, 15.0, 30.0
EPS_DEG = 0.01


def knob_deg(objects) -> float:
    """The knob's angle from its 0 degree stop, in degrees."""
    return math.degrees(float(objects["dial"].joints[g.JOINT]))


def error_deg(objects) -> float:
    return abs(knob_deg(objects) - TARGET_DEG)


def success(objects) -> bool:
    """The knob within 5 degrees of the reference; holds for 1 s (so it is not still turning)."""
    return error_deg(objects) <= FULL_DEG + EPS_DEG


def distance(objects) -> float:
    """Radians still to turn."""
    return math.radians(error_deg(objects))


def score(objects) -> float:
    """1.0 within 5 degrees of the reference, 0.5 within 15, 0.25 within 30, otherwise 0."""
    e = error_deg(objects)
    for limit, value in ((FULL_DEG, 1.0), (HALF_DEG, 0.5), (QUARTER_DEG, 0.25)):
        if e <= limit + EPS_DEG:
            return value
    return 0.0


# --- Scripted demo, left arm only. UNVERIFIED on the simulator: the grip height, the turn's pace and AIM_DEG. ---
# The gripper's yaw equals the bar's world yaw when the fingers close across the bar's 12 mm width (at yaw 0 they
# close along y, as in fragile-stow). A centred grip turns the knob with the wrist; the knob's hinge holds its centre.
TRAVEL = TOP + 0.100
GRIP_Z = TOP + g.PLATE_H + g.BAR_GAP + 0.004 + PAD_TIP_PAST_TCP  # pad tips 4 mm up the bar, 6 mm above the plate
SQUEEZE = 0.008
AIM_DEG = 0.0  # extra wrist turn to make up for lag; tune on the stack
TURN_S = 5.0  # 23 degrees a second


def demo():
    """Grip the knob across its bar, turn the wrist from -57.5 to +57.5 degrees over 5 s, let go and lift. 13.6 s of plan,
    then the 1 s hold."""
    x, y = KNOB_XY
    start = math.radians(DIAL_YAW_DEG)
    end = math.radians(DIAL_YAW_DEG + TARGET_DEG + AIM_DEG)
    grip = -(g.BAR[1] - SQUEEZE) / 2
    return {
        "left": [
            Waypoint(1.5, (x, y, TRAVEL), FINGER_OPEN, "over the knob", start),
            Waypoint(1.5, (x, y, GRIP_Z), FINGER_OPEN, "descend round the bar", start),
            Waypoint(0.6, (x, y, GRIP_Z), FINGER_OPEN, "settle", start),
            Waypoint(1.5, (x, y, GRIP_Z), grip, "close across the bar", start),
            Waypoint(TURN_S, (x, y, GRIP_Z), grip, "turn", end),
            Waypoint(0.8, (x, y, GRIP_Z), grip, "settle", end),
            Waypoint(1.5, (x, y, GRIP_Z), FINGER_OPEN, "release", end),
            Waypoint(1.2, (x, y, TRAVEL), FINGER_OPEN, "lift", end),
        ]
    }


def self_test(BodyState=None):
    """The scoring rules on hand-built joint angles; runs with scenekit's BodyState or any stand-in with ``joints``."""
    if BodyState is None:
        from scenekit import BodyState

    def at(deg):
        z = np.zeros(3)
        return {"dial": BodyState(pos=np.array([*KNOB_XY, TOP]), quat_xyzw=np.array(g.yaw_quat_xyzw(DIAL_YAW_DEG)),
                                  lin_vel=z, ang_vel=z, low=z, high=z, joints={g.JOINT: math.radians(deg)})}

    cases = {
        "start, against the 0 stop": (0.0, 0.0, False),
        "exactly on the reference": (115.0, 1.0, True),
        "4.9 degrees short": (110.1, 1.0, True),
        "5 degrees past": (120.0, 1.0, True),
        "6 degrees short": (109.0, 0.5, False),
        "15 degrees past": (130.0, 0.5, False),
        "snapped to the nearest tick, 135 degrees": (135.0, 0.25, False),
        "snapped to the other neighbouring tick, 90 degrees": (90.0, 0.25, False),
        "30 degrees past": (145.0, 0.25, False),
        "31 degrees short": (84.0, 0.0, False),
        "at the 270 stop": (270.0, 0.0, False),
        "turned the wrong way from the reference's world angle, 230 degrees": (230.0, 0.0, False),
    }
    for name, (deg, want, want_success) in cases.items():
        st = at(deg)
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)

    # Geometry: the reference pointer is inside the knob's range and its end-for-end reading is not; the plates don't
    # overlap; the bar clears the ticks when turned.
    assert g.LOWER_DEG < TARGET_DEG < g.UPPER_DEG and not (g.LOWER_DEG <= (TARGET_DEG + 180) % 360 <= g.UPPER_DEG)
    assert math.dist(KNOB_XY, REF_XY) > 2 * g.PLATE_R
    assert g.BAR[0] / 2 < g.TICK_R - g.TICK[0] / 2
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built angles, then the left arm's plan replayed through the
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
