"""Mecanum wheel kinematics.

Convention: x forward, y left, z counter-clockwise yaw.
Wheel order: front-left, front-right, rear-left, rear-right.
Check roller handedness and motor polarity on the actual chassis.
"""


def inverse_kinematics(vx, vy, wz, wheel_radius, half_length, half_width):
    if wheel_radius <= 0:
        raise ValueError("wheel_radius must be positive")
    k = half_length + half_width
    return (
        (vx - vy - k * wz) / wheel_radius,  # front-left
        (vx + vy + k * wz) / wheel_radius,  # front-right
        (vx + vy - k * wz) / wheel_radius,  # rear-left
        (vx - vy + k * wz) / wheel_radius,  # rear-right
    )


def forward_kinematics(w_fl, w_fr, w_rl, w_rr,
                       wheel_radius, half_length, half_width):
    k = half_length + half_width
    r = wheel_radius
    vx = r * (w_fl + w_fr + w_rl + w_rr) / 4.0
    vy = r * (-w_fl + w_fr + w_rl - w_rr) / 4.0
    wz = r * (-w_fl + w_fr - w_rl + w_rr) / (4.0 * k)
    return vx, vy, wz
