"""Thumbstick rotation of the tool while a hand is gripping (IK teleop loop, quest_dual_arm_teleop.py).

The operator should not have to twist the wrist for every tool rotation: while an arm is engaged, the driving
controller's thumbstick turns the tool slowly,

  stick Y (up = +)  tilt the tool tip up / down        axis  r = normalize(z_tool x up)   (stick up = aim up)
  stick X           --stick-x spin: about the tool's own Z (stick right = + rotation, the sense of twisting the hand
                                     right in the tool-frame mapping)
                    --stick-x yaw : about the base vertical, stick right = aim right (negative about +Z)

Pure functions (numpy / scipy only) so the shaping and the axes can be tested without a robot. The integration into
the engage reference orientation (arm.home_r) and the wind-up guard are in the teleop loop.
"""

import numpy as np

STICK_MAX_LEAD_DEG = 5.0   # integrate only while the commanded rotation is within this of what the last IK solution reaches
TILT_MIN_SIN = 0.2         # |z_tool x up| below this (tool within ~11.5 deg of vertical): tilt about the operator-right axis
UP = np.array([0.0, 0.0, 1.0])


def clean(raw) -> tuple:
    """(x, y) of one thumbstick, clipped to [-1, 1]; (0, 0) when missing, malformed or non-finite."""
    try:
        x, y = float(raw[0]), float(raw[1])
    except (TypeError, ValueError, IndexError, KeyError):
        return 0.0, 0.0
    if not (np.isfinite(x) and np.isfinite(y)):
        return 0.0, 0.0
    return float(np.clip(x, -1.0, 1.0)), float(np.clip(y, -1.0, 1.0))


def shape(s: float, deadzone: float) -> float:
    """Dead zone, then a quadratic curve (fine control near the centre): 0 inside the dead zone, +-1 at full deflection."""
    a = abs(s)
    if a <= deadzone:
        return 0.0
    return float(np.sign(s)) * float(np.clip((a - deadzone) / (1.0 - deadzone), 0.0, 1.0)) ** 2


def tilt_axis(z_tool, right_base) -> np.ndarray:
    """Unit axis whose positive rotation moves the tool Z toward up. Tool near vertical: the operator-right axis."""
    c = np.cross(z_tool, UP)
    n = float(np.linalg.norm(c))
    if n < TILT_MIN_SIN:
        return np.asarray(right_base, float) / np.linalg.norm(right_base)
    return c / n


def omega(stick, z_tool, right_base, x_mode="spin", signs=(1.0, 1.0), deadzone=0.15, rate_dps=15.0, max_dps=45.0):
    """Angular velocity [rad/s] in the arm base frame for one raw stick reading (x, y); zero vector when idle.
    Each axis reaches min(rate_dps, max_dps) at full deflection; the vector never exceeds max_dps."""
    sx, sy = clean(stick)
    sx = shape(sx, deadzone) * float(signs[0])
    sy = shape(sy, deadzone) * float(signs[1])
    full = np.deg2rad(min(rate_dps, max_dps))
    if full <= 0 or (sx == 0.0 and sy == 0.0):
        return np.zeros(3)
    z_tool = np.asarray(z_tool, float)
    axis_x = z_tool if x_mode == "spin" else -UP
    w = full * (sx * axis_x + sy * tilt_axis(z_tool, right_base))
    n, cap = float(np.linalg.norm(w)), np.deg2rad(max_dps)
    return w * (cap / n) if n > cap else w
