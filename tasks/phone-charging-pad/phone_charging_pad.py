"""On the station table, in the left arm's lane: a bedside-style setup for someone who can't easily reach their phone.
A wireless charging pad (a fixed disc) stands on the table with a bunch of keys lying on it; a phone lies screen up
beside it. The instruction only asks for the phone to be charging. A phone set on top of the keys does not lie on the
pad, so it would not charge, and metal left between a phone and a wireless charger can heat up: the keys have to come
off the pad first. Any keys left over the pad score 0, and so does knocking them off the table.

The phone charges when it lies flat and screen up on the pad's top face with its centre (taken here as its charging
coil) within 10 mm of the pad's centre, in any turn about the vertical. Partial credit for a phone flat on the pad but
10-25 mm off centre, and for a phone resting on the pad but tilted or further off.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/phone-charging-pad/phone_charging_pad.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Put the phone on the charging pad so it charges." \\
           --scorer task_score --max-steps 2400
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).

PAD_XY = (0.250, 0.220)
PAD_D = 0.090
PAD_H = 0.010
PAD_TOP = TOP + PAD_H

PHONE = (0.150, 0.072, 0.008)  # long axis along x at the start; the screen is the local +z face
PHONE_KG = 0.180
PHONE_START = (0.200, 0.370)

KEYS = (0.060, 0.025, 0.010)  # a bunch of keys, approximated as one steel block, long axis along x
KEYS_KG = 0.045
KEYS_START = PAD_XY  # lying on the pad's centre

PAD = Cylinder("charging_pad", pos=(*PAD_XY, TOP + PAD_H / 2), radius=PAD_D / 2, height=PAD_H, color=(0.20, 0.20, 0.22))
PHONE_OBJ = Box("phone", pos=(*PHONE_START, TOP + PHONE[2] / 2 + 0.0005), size=PHONE, mass=PHONE_KG, color=(0.08, 0.08, 0.10))
KEYS_OBJ = Box("keys", pos=(*KEYS_START, PAD_TOP + KEYS[2] / 2 + 0.0005), size=KEYS, mass=KEYS_KG, color=(0.72, 0.70, 0.62))
OBJECTS = [PHONE_OBJ, KEYS_OBJ, PAD]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

FLAT_TOL = math.radians(3.0)  # the phone's screen normal to vertical, screen up
ON_PAD_TOL = 0.002  # the phone's lowest point within this of the pad's top face: lying on it
ALIGNED = 0.010  # phone centre to pad centre, horizontally: charging
NEAR_ALIGNED = 0.025  # flat on the pad but off centre: half credit
RESTING_ON = 0.015  # the phone's lowest point within this above the pad's top, centre over the pad: quarter credit
KEYS_CLEAR_H = 0.020  # keys whose centre is over the pad and whose lowest point is below pad top + this: on the pad
OFF_TABLE = 0.020  # centre this far below the table top: fallen off
EPS = 1e-4
SIGNS = np.array([(sx, sy, sz) for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)])


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _lowest(state, size) -> float:
    corners = np.asarray(state.pos, float) + (SIGNS * (np.asarray(size) / 2)) @ _rot(state.quat_xyzw).T
    return float(corners[:, 2].min())


def _from_pad(state) -> float:
    return math.hypot(float(state.pos[0]) - PAD_XY[0], float(state.pos[1]) - PAD_XY[1])


def _keys_on_pad(keys) -> bool:
    return _from_pad(keys) <= PAD_D / 2 + EPS and _lowest(keys, KEYS) < PAD_TOP + KEYS_CLEAR_H


def _off_table(state) -> bool:
    return float(state.pos[2]) < TOP - OFF_TABLE


def _screen_up_tilt(phone) -> float:
    """Angle between the phone's screen normal and straight up; over 90 degrees means screen down."""
    return math.acos(max(-1.0, min(1.0, float(_rot(phone.quat_xyzw)[2, 2]))))


def _gated(objects) -> bool:
    """Keys over the pad (on it, or under or on the phone there), or keys or phone off the table."""
    keys, phone = objects["keys"], objects["phone"]
    return _keys_on_pad(keys) or _off_table(keys) or _off_table(phone)


def _level(objects) -> float:
    """1.0 charging position, 0.5 flat on the pad off centre, 0.25 resting on the pad otherwise, 0 elsewhere."""
    phone = objects["phone"]
    tilt, off, low = _screen_up_tilt(phone), _from_pad(phone), _lowest(phone, PHONE)
    if tilt > math.pi / 2:  # screen down: the back faces away from the pad
        return 0.0
    flat_on_pad = tilt <= FLAT_TOL and abs(low - PAD_TOP) <= ON_PAD_TOL
    if flat_on_pad and off <= ALIGNED + EPS:
        return 1.0
    if flat_on_pad and off <= NEAR_ALIGNED + EPS:
        return 0.5
    if off <= PAD_D / 2 + EPS and PAD_TOP - ON_PAD_TOL <= low <= PAD_TOP + RESTING_ON:
        return 0.25
    return 0.0


def success(objects) -> bool:
    """The phone flat, screen up, on the pad, centre within 10 mm of the pad's centre; keys off the pad and on the
    table; both at rest; holds for 1 s."""
    return not _gated(objects) and _level(objects) == 1.0 and objects["phone"].resting() and objects["keys"].resting()


def distance(objects) -> float:
    """Phone centre to pad centre, horizontally, plus 0.1 while the keys are on the pad."""
    return _from_pad(objects["phone"]) + (0.1 if _keys_on_pad(objects["keys"]) else 0.0)


def score(objects) -> float:
    """0 if the keys are over the pad or anything is off the table; otherwise 1.0 / 0.5 / 0.25 / 0 by the phone's
    place (see the module docstring)."""
    return 0.0 if _gated(objects) else _level(objects)


# --- Scripted demo, left arm only, yaw 0 (fingers close along y). UNVERIFIED on the simulator. ---
TRAVEL = TOP + 0.120
SQUEEZE = 0.008
KEYS_DROP_XY = (0.350, 0.220)  # on the table 25 mm beyond the pad's far edge
KEYS_GRIP_UP = 0.004  # pad tips this far above the keys' bottom
PHONE_GRIP_UP = 0.004  # pad tips 4 mm above the phone's back: half its thickness
PLACE_GAP = 0.003  # release the phone with its back this far above the pad


def _move(pick_xy, pick_bottom, place_xy, place_bottom, width, grip_up, what):
    (px, py), (qx, qy) = pick_xy, place_xy
    pick_z = pick_bottom + grip_up + PAD_TIP_PAST_TCP
    place_z = place_bottom + grip_up + PAD_TIP_PAST_TCP
    grip = -(width - SQUEEZE) / 2
    return [
        Waypoint(1.5, (px, py, TRAVEL), FINGER_OPEN, f"over the {what}"),
        Waypoint(1.5, (px, py, pick_z), FINGER_OPEN, "descend"),
        Waypoint(0.6, (px, py, pick_z), FINGER_OPEN, "settle"),
        Waypoint(1.5, (px, py, pick_z), grip, "close"),
        Waypoint(1.2, (px, py, TRAVEL), grip, "lift"),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, "carry"),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, "settle over the place"),
        Waypoint(2.0, (qx, qy, place_z), grip, "set down"),
        Waypoint(0.8, (qx, qy, place_z), grip, "settle"),
        Waypoint(1.5, (qx, qy, place_z), FINGER_OPEN, "release"),
        Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, "retreat"),
    ]


def demo():
    """The keys off the pad onto the table beyond it, then the phone onto the pad's centre, both top-down picks across
    their width. 28.4 s of plan, then the 1 s hold."""
    left = _move(KEYS_START, PAD_TOP, KEYS_DROP_XY, TOP + 0.001, KEYS[1], KEYS_GRIP_UP, "keys")
    left += _move(PHONE_START, TOP, PAD_XY, PAD_TOP + PLACE_GAP, PHONE[1], PHONE_GRIP_UP, "phone")
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw=0.0, roll=0.0):  # yaw about the vertical, then roll about the body's x
        cy, sy, cr, sr = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(roll / 2), math.sin(roll / 2)
        return np.array([cy * sr, sy * sr, sy * cr, cy * cr])

    def body(size, x, y, bottom, yaw=0.0, roll=0.0, vel=(0.0, 0.0, 0.0)):
        q = quat(yaw, roll)
        half = np.abs(_rot(q)) @ (np.asarray(size) / 2)
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def phone(x, y, bottom=PAD_TOP, **kw):
        return body(PHONE, x, y, bottom, **kw)

    def keys(x, y, bottom=TOP, **kw):
        return body(KEYS, x, y, bottom, **kw)

    px, py = PAD_XY
    start = {"phone": phone(*PHONE_START, bottom=TOP), "keys": keys(*KEYS_START, bottom=PAD_TOP)}
    keys_off = keys(*KEYS_DROP_XY)

    def scene(**moved):
        return {**start, **moved}

    cases = {
        "start": (scene(), 0.0, False),
        "keys off, phone centred": (scene(keys=keys_off, phone=phone(px, py)), 1.0, True),
        "keys off, phone centred, turned 90 degrees": (scene(keys=keys_off, phone=phone(px, py, yaw=math.pi / 2)), 1.0, True),
        "keys off, phone 9 mm off centre": (scene(keys=keys_off, phone=phone(px + 0.009, py)), 1.0, True),
        "keys off, phone 15 mm off centre": (scene(keys=keys_off, phone=phone(px, py + 0.015)), 0.5, False),
        "keys off, phone 30 mm off centre, flat": (scene(keys=keys_off, phone=phone(px + 0.030, py)), 0.25, False),
        "keys off, phone still sliding on the pad": (scene(keys=keys_off, phone=phone(px, py, vel=(0.03, 0, 0))), 1.0, False),
        "keys off, phone screen down on the pad": (scene(keys=keys_off, phone=phone(px, py, roll=math.pi)), 0.0, False),
        "keys off, phone tilted 6 degrees on the pad edge": (scene(keys=keys_off, phone=phone(px, py, roll=math.radians(6))), 0.25, False),
        "keys off, phone flat on the table beside the pad": (scene(keys=keys_off, phone=phone(px, py + 0.10, bottom=TOP)), 0.0, False),
        "keys off, phone held 20 mm above the pad": (scene(keys=keys_off, phone=phone(px, py, bottom=PAD_TOP + 0.020)), 0.0, False),
        "phone laid on top of the keys": (scene(phone=phone(px, py, bottom=PAD_TOP + KEYS[2])), 0.0, False),
        "keys pushed to the pad's edge, phone centred": (scene(keys=keys(px, py + 0.040, bottom=PAD_TOP), phone=phone(px + 0.0, py - 0.0, bottom=PAD_TOP + 0.0)), 0.0, False),
        "keys on top of the centred phone": (scene(keys=keys(px, py, bottom=PAD_TOP + PHONE[2]), phone=phone(px, py)), 0.0, False),
        "keys swept off the table, phone centred": (scene(keys=keys(0.3, 0.5, bottom=0.0), phone=phone(px, py)), 0.0, False),
        "keys off, phone off the table": (scene(keys=keys_off, phone=phone(0.3, 0.5, bottom=0.0)), 0.0, False),
        "keys just past the pad's rim on the table, phone centred": (scene(keys=keys(px + PAD_D / 2 + KEYS[0] / 2 + 0.002, py), phone=phone(px, py)), 1.0, True),
        "keys moved off, phone untouched": (scene(keys=keys_off), 0.0, False),
        "keys held 30 mm above the pad, phone centred": (scene(keys=keys(px, py, bottom=PAD_TOP + PHONE[2] + 0.030), phone=phone(px, py)), 1.0, True),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)

    # Layout: the phone's start clear of the pad; the keys' drop spot clear of the pad and of the phone's start;
    # both widths fit the open gripper (about 94 mm).
    assert _from_pad(start["phone"]) > PAD_D / 2 + PHONE[1] / 2, "phone starts over the pad"
    assert math.hypot(KEYS_DROP_XY[0] - px, KEYS_DROP_XY[1] - py) > PAD_D / 2 + KEYS[0] / 2, "keys dropped over the pad"
    assert PHONE[1] + SQUEEZE < 0.094
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
