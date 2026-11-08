"""Dimensions shared by the task file and the dial's USD generator (pure Python, no simulator).

Dial frame: origin at the centre of the dial plate's underside, on the table; z up; the knob's 0 degree position
points along the frame's +x, and the knob turns counter-clockwise (seen from above) towards its 270 degree stop.
"""

import math

PLATE_R = 0.045  # dial plate: a disc 90 mm across
PLATE_H = 0.010  # and 10 mm tall; the knob turns on its top face
BAR = (0.056, 0.012, 0.018)  # the knob: a bar 56 mm long, 12 mm wide, 18 mm tall, centred on the axis
BAR_GAP = 0.002  # the bar's underside clears the plate's top by this much
TIP = (0.010, 0.012, 0.002)  # a red pointer tip on top of the bar at its +x end
TICK = (0.006, 0.0015, 0.001)  # tick marks: 6 mm radial, 1.5 mm wide, 1 mm tall, on the plate's top face
TICK_R = 0.041  # tick centres this far from the axis
TICK_DEG = tuple(range(0, 271, 45))  # 0, 45, ..., 270
LOWER_DEG, UPPER_DEG = 0.0, 270.0  # the knob's stops

PLATE_COLOR = (0.85, 0.85, 0.83)
TICK_COLOR = (0.10, 0.10, 0.10)
KNOB_COLOR = (0.15, 0.35, 0.85)  # the turnable knob is blue
REF_COLOR = (0.10, 0.10, 0.10)  # the reference dial's pointer is black
TIP_COLOR = (0.85, 0.10, 0.10)

JOINT = "dial_joint"
KNOB_MASS = 0.030
BASE_MASS = 1.0

BAR_CZ = PLATE_H + BAR_GAP + BAR[2] / 2  # bar centre height above the table
TIP_CX = BAR[0] / 2 - TIP[0] / 2  # tip centre along the bar from the axis
TIP_CZ = PLATE_H + BAR_GAP + BAR[2] + TIP[2] / 2


def yaw_quat_wxyz(deg):
    h = math.radians(deg) / 2
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def yaw_quat_xyzw(deg):
    w, x, y, z = yaw_quat_wxyz(deg)
    return (x, y, z, w)
