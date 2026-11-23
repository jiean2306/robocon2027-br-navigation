#!/usr/bin/env python3
"""
Multi-Level TF Tree, Mecanum Kinematics, 3-Stage Wheel-Lifting Step-Climber,
Path Follower, 3D Arena Visualizer & Analytic 2D LiDAR Raycast Bridge
for ABU Robocon 2027 Builder Robot (BR v7 Fusion 360 Design).

Responsibilities
----------------
1. Publishes the complete ROS 2 TF tree:
       map -> odom -> base_link -> lidar_link
2. Supports starting at the Ground Floor Start/Retry Zone (-4.35, -5.15, z=0.00 m)
   AND transitioning across Ground (0.00 m) <-> Level 1 (+0.60 m) <-> Level 2 (+0.90 m)
   via:
     * Automatic ramp/stair elevation tracking when driving over Ramps or Stairs
     * 3-Stage Wheel-Lifting Step-Climb State Machine modeled after the Fusion 360
       BR v7 mechanism (`Guide_Front/Rear`, `Caster_Front/Rear_L/R`, dual vertical
       lift columns, and under-chassis support rollers).
3. Accepts both Manual Teleop (/cmd_vel from Joystick GUI, WASD, or Arrow Keys) and
   Autonomous A* Path Following (/br_planned_path).
4. Publishes the complete 3D ABU Robocon 2027 Arena (/br_arena_markers) and
   level-aware 360-degree 2D LiDAR scans (/scan).
"""

from __future__ import annotations

import math
import threading
import time
from typing import List, Optional, Tuple

try:
    import gz.transport13 as gzt
    from gz.msgs10 import boolean_pb2 as gz_boolean_pb2
    from gz.msgs10 import pose_pb2 as gz_pose_pb2

    GZ_POSE_SYNC_AVAILABLE = True
except Exception:
    gzt = None
    gz_boolean_pb2 = None
    gz_pose_pb2 = None
    GZ_POSE_SYNC_AVAILABLE = False

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from br_nav_sandbox.arena_geometry import build_exact_arena_primitives, rgb_norm


Segment2D = Tuple[float, float, float, float]


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert Euler angles (roll, pitch, yaw) in radians to quaternion (x, y, z, w)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (qx, qy, qz, qw)


def mecanum_inverse_kinematics(
    vx: float,
    vy: float,
    wz: float,
    wheel_radius: float = 0.075,
    half_length: float = 0.25,
    half_width: float = 0.25,
) -> Tuple[float, float, float, float]:
    """Compute (w_fl, w_fr, w_rl, w_rr) in rad/s from body twist (vx, vy, wz)."""
    if wheel_radius <= 0.0:
        raise ValueError('wheel_radius must be positive')
    k = half_length + half_width
    return (
        (vx - vy - k * wz) / wheel_radius,
        (vx + vy + k * wz) / wheel_radius,
        (vx + vy - k * wz) / wheel_radius,
        (vx - vy + k * wz) / wheel_radius,
    )


def mecanum_forward_kinematics(
    w_fl: float,
    w_fr: float,
    w_rl: float,
    w_rr: float,
    wheel_radius: float = 0.075,
    half_length: float = 0.25,
    half_width: float = 0.25,
) -> Tuple[float, float, float]:
    """Compute body twist (vx, vy, wz) from wheel speeds (w_fl, w_fr, w_rl, w_rr)."""
    k = half_length + half_width
    r = wheel_radius
    vx = r * (w_fl + w_fr + w_rl + w_rr) / 4.0
    vy = r * (-w_fl + w_fr + w_rl - w_rr) / 4.0
    wz = r * (-w_fl + w_fr - w_rl + w_rr) / (4.0 * k)
    return vx, vy, wz


def _box_segments(cx: float, cy: float, sx: float, sy: float) -> List[Segment2D]:
    """Return the 4 bounding line segments of an axis-aligned box centered at (cx, cy)."""
    hx = 0.5 * sx
    hy = 0.5 * sy
    x0, x1 = cx - hx, cx + hx
    y0, y1 = cy - hy, cy + hy
    return [
        (x0, y0, x1, y0),
        (x1, y0, x1, y1),
        (x1, y1, x0, y1),
        (x0, y1, x0, y0),
    ]


def _cylinder_segments(cx: float, cy: float, radius: float, n: int = 12) -> List[Segment2D]:
    """Return an n-sided polygon approximation of a cylinder centered at (cx, cy)."""
    segs: List[Segment2D] = []
    for i in range(n):
        a1 = 2.0 * math.pi * i / n
        a2 = 2.0 * math.pi * ((i + 1) % n) / n
        segs.append(
            (
                cx + radius * math.cos(a1),
                cy + radius * math.sin(a1),
                cx + radius * math.cos(a2),
                cy + radius * math.sin(a2),
            )
        )
    return segs


def build_l1_arena_wall_segments(include_transfer_blocks: bool = False) -> List[Segment2D]:
    """
    Return 2D line segments (x1, y1, x2, y2) representing the physical vertical
    walls, barriers, and game blocks encountered by a 2D LiDAR on Level 1 (z = +0.60 m).
    """
    segments: List[Segment2D] = [
        # L1 North & South perimeter barriers (inner face at y = +-2.97 m)
        (-2.97, 2.97, 2.97, 2.97),
        (-2.97, -2.97, 2.97, -2.97),
        # L1 West perimeter barrier: lower (y in [-2.97, -1.80]) & upper (y in [-0.80, 2.97])
        (-2.97, -2.97, -2.97, -1.80),
        (-2.97, -0.80, -2.97, 2.97),
        # L1 East perimeter barrier: lower (y in [-2.97, -1.80]) & upper (y in [-0.80, 2.97])
        (2.97, -2.97, 2.97, -1.80),
        (2.97, -0.80, 2.97, 2.97),
        # Red Transfer Area enclosure walls (x in [-4.00, -2.97], y in [-1.80, -0.80])
        (-4.00, -1.80, -2.97, -1.80),
        (-4.00, -0.80, -2.97, -0.80),
        (-4.00, -1.80, -4.00, -0.80),
        # Blue Transfer Area enclosure walls (x in [2.97, 4.00], y in [-1.80, -0.80])
        (2.97, -1.80, 4.00, -1.80),
        (2.97, -0.80, 4.00, -0.80),
        (4.00, -1.80, 4.00, -0.80),
        # Central Level 2 platform step walls (3000 x 3000 mm, +300 mm tall)
        (-1.50, -1.50, 1.50, -1.50),
        (1.50, -1.50, 1.50, 1.50),
        (1.50, 1.50, -1.50, 1.50),
        (-1.50, 1.50, -1.50, -1.50),
        # Level 1 Center Divider Fences (30 mm thick x 720 mm long North & South of L2)
        (-0.015, -2.22, -0.015, -1.50),
        (0.015, -2.22, 0.015, -1.50),
        (-0.015, 1.50, -0.015, 2.22),
        (0.015, 1.50, 0.015, 2.22),
    ]
    if include_transfer_blocks:
        # Sample delivered Earth Block (350x350mm) and Sky Block (200x200mm) in Transfer Area
        segments.extend(_box_segments(-3.72, -1.05, 0.35, 0.35))
        segments.extend(_box_segments(-3.75, -1.55, 0.20, 0.20))
    return segments


def build_ground_arena_wall_segments() -> List[Segment2D]:
    """
    Return 2D line segments visible to the 2D LiDAR when BR is on the Ground Area (z = 0.0 m),
    including:
      * Outer 11.0 x 11.0 m Game Field Boundary fence
      * 4 Ground Center Divider fence segments
      * Level 1 (6.0 x 6.0 m) side walls + Stairs / Transfer Area / Ramp walls
      * Mustika Pillar (diameter 270 mm at (0.0, +4.25))
      * All 12 Sky Blocks (200 x 200 mm cubes in the 5x5 Ground Shared Area grid at (0.0, -4.25))
      * All 20 Red Earth Blocks + 20 Blue Earth Blocks (350 x 350 mm cubes in the Storage Areas)
    """
    segs: List[Segment2D] = [
        # Outer 11.0 x 11.0 m Game Field Boundary
        (-5.50, -5.50, 5.50, -5.50),
        (5.50, -5.50, 5.50, 5.50),
        (5.50, 5.50, -5.50, 5.50),
        (-5.50, 5.50, -5.50, -5.50),
        # Ground Center Dividers (South 650mm x 2, North 750mm x 2)
        (0.0, -5.50, 0.0, -4.85),
        (0.0, -3.65, 0.0, -3.00),
        (0.0, 3.00, 0.0, 3.75),
        (0.0, 4.75, 0.0, 5.50),
        # Level 1 outer side walls (6.0 x 6.0 m) + Stairs/Transfer/Ramp outer walls
        (-3.00, -3.00, 3.00, -3.00),
        (-3.00, 3.00, 3.00, 3.00),
        (-3.00, -3.00, -3.00, -2.70),
        (-3.00, 2.70, -3.00, 3.00),
        (3.00, -3.00, 3.00, -2.70),
        (3.00, 2.70, 3.00, 3.00),
        (-4.00, -2.70, -3.00, -2.70),
        (-4.00, 2.70, -3.00, 2.70),
        (-4.00, -2.70, -4.00, 2.70),
        (3.00, -2.70, 4.00, -2.70),
        (3.00, 2.70, 4.00, 2.70),
        (4.00, -2.70, 4.00, 2.70),
    ]

    # Mustika Pillar (diameter 270 mm, radius 0.135 m at (0.0, +4.25))
    segs.extend(_cylinder_segments(0.0, 4.25, 0.135, n=12))

    # 12 Sky Blocks (200 x 200 mm) in the 5x5 Ground Shared Area grid at (0.0, -4.25)
    gx, gy = 0.0, -4.25
    sky_cells = [
        (0, 0), (0, 2), (0, 4),
        (1, 1), (1, 3),
        (2, 0), (2, 4),
        (3, 1), (3, 3),
        (4, 0), (4, 2), (4, 4),
    ]
    for row, col in sky_cells:
        sx = gx - 0.50 + (col + 0.5) * 0.20
        sy = gy + 0.50 - (row + 0.5) * 0.20
        segs.extend(_box_segments(sx, sy, 0.20, 0.20))

    # 20 Red + 20 Blue Earth Blocks (bottom layer of 10 in each Storage Area: 2 rows x 5 cols of 350x350 mm)
    side = 0.35
    pitch = 0.375
    for cx in (-4.50, 4.50):
        for local in range(10):
            row = local // 5
            col = local % 5
            ex = cx + (col - 2) * pitch
            ey = 5.00 + (row - 0.5) * pitch
            segs.extend(_box_segments(ex, ey, side, side))

    return segs


def build_l2_arena_wall_segments() -> List[Segment2D]:
    """Return 2D line segments visible to the 2D LiDAR when BR is on Level 2 (z = 0.90 m)."""
    return _cylinder_segments(0.0, 0.0, 0.135, n=12)


def point_to_segment_distance(px: float, py: float, seg: Segment2D) -> float:
    """Return shortest Euclidean distance from point (px, py) to 2D line segment."""
    x1, y1, x2, y2 = seg
    dx = x2 - x1
    dy = y2 - y1
    len2 = dx * dx + dy * dy
    if len2 < 1e-12:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / len2))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def is_circle_colliding(
    px: float, py: float, radius: float, segments: List[Segment2D]
) -> bool:
    """Return True if a circle of `radius` at (px, py) intersects any 2D wall/obstacle segment."""
    for seg in segments:
        if point_to_segment_distance(px, py, seg) < radius:
            return True
    return False


def wrap_angle(angle: float) -> float:
    """Normalize angle in radians to [-pi, +pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def interpolate_closed_waypoints(
    waypoints: List[Tuple[float, float, float]], progress_s: float, speed_mps: float = 0.95
) -> Tuple[float, float, float]:
    """
    Smoothly interpolate (x, y, z) along a closed 3D waypoint loop at constant linear speed.
    Automatically handles Ramp z-slopes linearly between consecutive waypoints.
    """
    n = len(waypoints)
    if n == 0:
        return (0.0, 0.0, 0.0)
    if n == 1:
        return waypoints[0]

    seg_lens: List[float] = []
    total_len = 0.0
    for i in range(n):
        p0 = waypoints[i]
        p1 = waypoints[(i + 1) % n]
        d = math.sqrt((p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2 + (p1[2] - p0[2]) ** 2)
        seg_lens.append(max(1e-4, d))
        total_len += seg_lens[-1]

    dist = (progress_s * speed_mps) % total_len
    acc = 0.0
    for i, slen in enumerate(seg_lens):
        if acc + slen >= dist:
            u = (dist - acc) / slen
            p0 = waypoints[i]
            p1 = waypoints[(i + 1) % n]
            return (
                p0[0] + u * (p1[0] - p0[0]),
                p0[1] + u * (p1[1] - p0[1]),
                p0[2] + u * (p1[2] - p0[2]),
            )
        acc += slen
    return waypoints[0]


def get_match_robot_waypoints() -> dict[str, List[Tuple[float, float, float]]]:
    """
    Rulebook V1.1 Section 3.2 & 4.3 compliant patrol loops for the 3 other match robots:
      * our_tr   : Red Team Transporter Robot (Red Ground + Ground Shared + Red Ramp + Red Transfer Area)
      * enemy_br : Blue Team Builder Robot (Blue Transfer + Blue L1 + L1 Shared Areas + Level 2 Shared Area)
      * enemy_tr : Blue Team Transporter Robot (Blue Ground + Ground Shared + Blue Ramp + Blue Transfer Area)
    """
    return {
        'our_tr': [
            (-4.85, -4.40, 0.00),  # Red Ground Start approach
            (-1.15, -4.25, 0.00),  # South Ground Shared Area (Sky Block harvest)
            (-4.75, -3.50, 0.00),  # Red Ground West corridor
            (-4.50, 3.85, 0.00),   # Red Storage Area (Earth Block harvest)
            (-3.50, 2.70, 0.00),   # Red Ramp North entry (z = 0.00 m)
            (-3.50, -0.80, 0.60),  # Red Ramp top (z = 0.60 m)
            (-3.50, -1.30, 0.60),  # Red Transfer Area delivery (z = 0.60 m)
            (-3.50, -0.80, 0.60),  # Exit Red Transfer Area
            (-3.50, 2.70, 0.00),   # Descend Red Ramp back to Ground
            (-4.75, 2.70, 0.00),   # Red Ground West lane return
        ],
        'enemy_br': [
            (3.45, -1.30, 0.60),   # Blue Transfer Area pickup
            (2.20, -1.30, 0.60),   # Blue L1 East corridor
            (2.20, -2.45, 0.60),   # Blue L1 South-East corner
            (0.15, -2.55, 0.60),   # South L1 Shared Area (contested Building Spot!)
            (2.20, -2.45, 0.60),   # Return to Blue L1
            (2.20, 2.45, 0.60),    # Blue L1 North-East corner
            (0.15, 2.55, 0.60),    # North L1 Shared Area (contested Building Spot!)
            (2.20, 2.45, 0.60),    # Return to Blue L1
            (2.05, 0.00, 0.60),    # Blue L1 East edge of Level 2
            (0.95, 0.00, 0.90),    # Climb +300mm onto Level 2 Shared Area!
            (0.90, 0.90, 0.90),    # L2 NE Building Spot
            (-0.90, 0.90, 0.90),   # L2 NW Building Spot (contested!)
            (-0.90, -0.90, 0.90),  # L2 SW Building Spot (contested!)
            (0.90, -0.90, 0.90),   # L2 SE Building Spot
            (2.05, -1.30, 0.60),   # Descend back to Blue L1
        ],
        'enemy_tr': [
            (4.85, -4.40, 0.00),   # Blue Ground Start approach
            (1.15, -4.25, 0.00),   # South Ground Shared Area (Sky Block harvest)
            (4.75, -3.50, 0.00),   # Blue Ground East corridor
            (4.50, 3.85, 0.00),    # Blue Storage Area (Earth Block harvest)
            (3.50, 2.70, 0.00),    # Blue Ramp North entry (z = 0.00 m)
            (3.50, -0.80, 0.60),   # Blue Ramp top (z = 0.60 m)
            (3.50, -1.30, 0.60),   # Blue Transfer Area delivery (z = 0.60 m)
            (3.50, -0.80, 0.60),   # Exit Blue Transfer Area
            (3.50, 2.70, 0.00),    # Descend Blue Ramp back to Ground
            (4.75, 2.70, 0.00),    # Blue Ground East lane return
        ],
    }


def ray_segment_intersection(
    ox: float,
    oy: float,
    dx: float,
    dy: float,
    seg: Segment2D,
    max_range: float,
) -> Optional[float]:
    """
    Compute intersection distance t in (0, max_range] between ray (ox, oy) + t*(dx, dy)
    and 2D line segment (x1, y1)-(x2, y2).
    """
    x1, y1, x2, y2 = seg
    sx = x2 - x1
    sy = y2 - y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-12:
        return None
    qpx = x1 - ox
    qpy = y1 - oy
    t = (qpx * sy - qpy * sx) / denom
    u = (qpx * dy - qpy * dx) / denom
    if 0.0 <= u <= 1.0 and 0.02 <= t <= max_range:
        return t
    return None


def simulate_l1_laser_scan(
    sensor_x: float,
    sensor_y: float,
    sensor_yaw: float,
    num_beams: int = 360,
    range_min: float = 0.08,
    range_max: float = 8.0,
    segments: Optional[List[Segment2D]] = None,
) -> Tuple[float, float, List[float]]:
    """
    Synthesize a 360-degree 2D LaserScan at (sensor_x, sensor_y, sensor_yaw) on Level 1.
    Returns (angle_min, angle_increment, ranges).
    """
    segs = segments if segments is not None else build_l1_arena_wall_segments()
    angle_min = -math.pi
    angle_inc = (2.0 * math.pi) / float(num_beams)
    ranges: List[float] = []

    for i in range(num_beams):
        beam_angle = sensor_yaw + angle_min + i * angle_inc
        dx = math.cos(beam_angle)
        dy = math.sin(beam_angle)
        best = range_max
        for seg in segs:
            hit = ray_segment_intersection(sensor_x, sensor_y, dx, dy, seg, best)
            if hit is not None and hit < best:
                best = hit
        if best < range_min:
            best = range_min
        ranges.append(best)

    return angle_min, angle_inc, ranges


def compute_terrain_elevation(x: float, y: float, current_z: float) -> Tuple[float, float, str]:
    """
    Determine natural arena surface elevation z (m), pitch angle (rad), and zone label
    at planar coordinate (x, y) given current robot elevation `current_z`:
      * Level 2 platform (|x| <= 1.50, |y| <= 1.50): z = 0.90 m (when climbed/lifted)
      * Level 1 platform (|x| <= 2.97, |y| <= 2.97): z = 0.60 m (when on L1)
      * Red/Blue Transfer Area (3.00 <= |x| <= 4.00, -1.80 <= y <= -0.80): z = 0.60 m
      * Red/Blue Ramp (3.00 <= |x| <= 4.00, -0.80 < y <= 2.70):
            smooth linear slope from z = 0.00 m at y = +2.70 m to z = +0.60 m at y = -0.80 m
      * Red/Blue Stairs (3.00 <= |x| <= 4.00, -2.70 <= y < -1.80):
            3 steps of 0.15 m, 0.30 m, 0.45 m
      * Ground Area: z = 0.00 m
    """
    # Check Red or Blue Ramp (3500 mm long, rising from y=+2.70 [z=0] to y=-0.80 [z=0.60])
    if 2.97 <= abs(x) <= 4.05 and -0.80 < y <= 2.70:
        frac = max(0.0, min(1.0, (2.70 - y) / 3.50))
        z_ramp = 0.60 * frac
        return z_ramp, -0.1698, f'RAMP ({z_ramp:.2f}m)'

    # Check Red or Blue Stairs (3 steps between y=-2.70 and y=-1.80)
    if 2.97 <= abs(x) <= 4.05 and -2.70 <= y < -1.80:
        if y < -2.40:
            return 0.15, 0.0, 'STAIR STEP 1 (+0.15m)'
        if y < -2.10:
            return 0.30, 0.0, 'STAIR STEP 2 (+0.30m)'
        return 0.45, 0.0, 'STAIR STEP 3 (+0.45m)'

    # Check Red or Blue Transfer Area
    if 2.97 <= abs(x) <= 4.05 and -1.80 <= y <= -0.80:
        return 0.60, 0.0, 'TRANSFER AREA (+0.60m)'

    # Check Level 2 platform (3.0 x 3.0 m)
    if abs(x) <= 1.50 and abs(y) <= 1.50:
        if current_z >= 0.72:
            return 0.90, 0.0, 'LEVEL 2 (+0.90m)'
        return 0.60, 0.0, 'AT L2 STEP WALL (Press [L] to Wheel-Lift Climb!)'

    # Check Level 1 platform (6.0 x 6.0 m)
    if abs(x) <= 2.98 and abs(y) <= 2.98:
        if current_z >= 0.25:
            return 0.60, 0.0, 'LEVEL 1 (+0.60m)'
        return 0.00, 0.0, 'AT L1 WALL (Drive via Ramp/Stairs or Press [L] to Lift!)'

    return 0.00, 0.0, 'GROUND AREA (0.00m)'


class L1SimBridgeNode(Node):
    """
    ROS 2 Jazzy node providing the multi-level TF tree (map -> odom -> base_link -> lidar_link),
    mecanum kinematics odometry, 3-stage wheel-lifting step-climbing mechanism,
    manual/autonomous control mode switching, 3D ABU Robocon 2027 arena visualization
    (/br_arena_markers), and 2D LiDAR scanning (/scan).
    """

    def __init__(self) -> None:
        super().__init__('l1_sim_bridge')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'lidar_link')
        # Default start at Red Ground Start/Retry Zone (-4.35, -5.15, z=0.00) facing North (+Y)
        self.declare_parameter('initial_x', -4.35)
        self.declare_parameter('initial_y', -5.15)
        self.declare_parameter('initial_z', 0.00)
        self.declare_parameter('initial_yaw', 1.5707963)
        self.declare_parameter('lidar_offset_x', 0.0)
        self.declare_parameter('lidar_offset_y', 0.0)
        self.declare_parameter('lidar_offset_z', 0.25)
        self.declare_parameter('publish_internal_scan', True)
        self.declare_parameter('publish_internal_odom', True)
        self.declare_parameter('auto_follow_path', False)
        self.declare_parameter('follow_speed_mps', 0.65)
        self.declare_parameter('num_beams', 360)
        self.declare_parameter('range_min', 0.08)
        self.declare_parameter('range_max', 8.0)

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.z = float(self.get_parameter('initial_z').value)
        self.pitch = 0.0
        self.yaw = float(self.get_parameter('initial_yaw').value)

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self._last_time = self.get_clock().now()

        self._l1_segments = build_l1_arena_wall_segments()
        self._ground_segments = build_ground_arena_wall_segments()
        self._l2_segments = build_l2_arena_wall_segments()

        # Control mode & 3-Stage Wheel-Lifting Climb state machine
        self._auto_mode: bool = bool(self.get_parameter('auto_follow_path').value)
        self._waypoints: List[Tuple[float, float]] = []
        self._wp_index: int = 0

        self._climb_active: bool = False
        self._climb_start_time: float = 0.0
        self._climb_duration: float = 1.8
        self._climb_start_pose: Tuple[float, float, float] = (self.x, self.y, self.z)
        self._climb_target_pose: Tuple[float, float, float] = (self.x, self.y, self.z)
        self._climb_stage_text: str = 'IDLE (4 Wheels Locked)'
        self._zone_label: str = 'GROUND START ZONE (0.00m)'
        self._min_lidar_dist: float = 8.0
        self._safety_brake_active: bool = False
        self._collision_radius: float = 0.31
        self._enemy_collision_segments: List[Segment2D] = []

        # TF broadcasters
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._publish_static_transforms()

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Subscriptions & Publishers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.climb_sub = self.create_subscription(
            String, '/br_climb_command', self._climb_cmd_cb, 10
        )
        self.path_sub = self.create_subscription(
            Path, '/br_planned_path', self._path_cb, latched_qos
        )
        self.gz_odom_sub = self.create_subscription(
            Odometry, '/gz_odom', self._gz_odom_cb, 10
        )
        self.tr_override_sub = self.create_subscription(
            PointStamped, '/br_tr_pose_override', self._tr_override_cb, 10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.arena_pub = self.create_publisher(MarkerArray, '/br_arena_markers', latched_qos)
        self.enemy_pub = self.create_publisher(MarkerArray, '/br_enemy_markers', 10)
        self.status_pub = self.create_publisher(String, '/br_climb_status', 10)

        self._external_gz_odom_received = False
        self._gz_odom_ignore_until: float = 0.0
        self._gz_last_pose_sync: float = 0.0
        self._gz_node = gzt.Node() if GZ_POSE_SYNC_AVAILABLE else None
        self._tr_override_active: bool = False
        self._last_scan_meta: Optional[Tuple[float, float, List[float]]] = None

        # 3 Autonomous Match Robots (Rule 11.1): OUR TR, ENEMY BR, ENEMY TR
        self._match_robots_enabled: bool = True
        self._match_robots_moving: bool = True
        self._match_time_s: float = 0.0
        self._match_waypoints = get_match_robot_waypoints()
        self._our_tr_pose: Tuple[float, float, float] = (-4.85, -4.40, 0.00)
        self._enemy_br_pose: Tuple[float, float, float] = (0.15, -2.55, 0.60)
        self._enemy_tr_pose: Tuple[float, float, float] = (4.85, -4.40, 0.00)

        # Publish the 3D ABU Robocon 2027 Arena MarkerArray immediately and at 0.5 Hz
        self._arena_markers_msg = self._build_arena_marker_array()
        self.arena_pub.publish(self._arena_markers_msg)
        self.arena_timer = self.create_timer(2.0, self._republish_arena_markers)

        # 20 Hz kinematics + wheel-lift climber + TF timer, 10 Hz LiDAR scan timer
        self.odom_timer = self.create_timer(0.05, self._step_kinematics_and_tf)
        self.scan_timer = self.create_timer(0.10, self._publish_scan)

        self.get_logger().info(
            f'L1SimBridgeNode started at Ground Start/Retry Zone ({self.x:.2f}, {self.y:.2f}, z={self.z:.2f}). '
            f'Wheel-Lifting mechanism & Manual/Auto control ready.'
        )

    def _build_arena_marker_array(self) -> MarkerArray:
        """Build 3D visualization markers for the exact ABU Robocon 2027 arena."""
        now = self.get_clock().now().to_msg()
        map_frame = str(self.get_parameter('map_frame').value)
        m_array = MarkerArray()

        for idx, prim in enumerate(build_exact_arena_primitives(include_game_objects=True)):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = map_frame
            m.ns = 'robocon_2027_arena'
            m.id = idx
            m.action = Marker.ADD

            x, y, z, roll, pitch, yaw = prim.pose
            qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(z)
            m.pose.orientation.x = qx
            m.pose.orientation.y = qy
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw

            if prim.shape == 'box':
                m.type = Marker.CUBE
                m.scale.x = float(prim.size[0])
                m.scale.y = float(prim.size[1])
                m.scale.z = float(prim.size[2])
            elif prim.shape == 'cylinder':
                m.type = Marker.CYLINDER
                m.scale.x = float(prim.size[0] * 2.0)
                m.scale.y = float(prim.size[0] * 2.0)
                m.scale.z = float(prim.size[1])
            elif prim.shape == 'sphere':
                m.type = Marker.SPHERE
                m.scale.x = float(prim.size[0] * 2.0)
                m.scale.y = float(prim.size[0] * 2.0)
                m.scale.z = float(prim.size[0] * 2.0)

            cr, cg, cb = rgb_norm(prim.color_key)
            m.color.r = float(cr)
            m.color.g = float(cg)
            m.color.b = float(cb)
            m.color.a = 1.0
            m_array.markers.append(m)

        return m_array

    def _republish_arena_markers(self) -> None:
        self.arena_pub.publish(self._arena_markers_msg)

    def _publish_static_transforms(self) -> None:
        now = self.get_clock().now().to_msg()
        map_frame = str(self.get_parameter('map_frame').value)
        odom_frame = str(self.get_parameter('odom_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        lidar_frame = str(self.get_parameter('lidar_frame').value)

        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = map_frame
        t_map_odom.child_frame_id = odom_frame
        t_map_odom.transform.rotation.w = 1.0

        t_base_lidar = TransformStamped()
        t_base_lidar.header.stamp = now
        t_base_lidar.header.frame_id = base_frame
        t_base_lidar.child_frame_id = lidar_frame
        t_base_lidar.transform.translation.x = float(self.get_parameter('lidar_offset_x').value)
        t_base_lidar.transform.translation.y = float(self.get_parameter('lidar_offset_y').value)
        t_base_lidar.transform.translation.z = float(self.get_parameter('lidar_offset_z').value)
        t_base_lidar.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform([t_map_odom, t_base_lidar])

    def _start_wheel_lift_climb(self, target_x: float, target_y: float, target_z: float) -> None:
        """Initiate the 3-stage Fusion 360 wheel-lifting step-climb transition."""
        self._climb_active = True
        self._climb_start_time = self.get_clock().now().nanoseconds * 1e-9
        self._climb_start_pose = (self.x, self.y, self.z)
        self._climb_target_pose = (target_x, target_y, target_z)
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

    def _climb_cmd_cb(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if cmd == 'mode_manual':
            self._auto_mode = False
            self.vx = self.vy = self.wz = 0.0
            self.get_logger().info('Switched to MANUAL Control Mode (Joystick / WASD / Arrows).')
            return
        if cmd == 'mode_auto':
            self._auto_mode = True
            self.get_logger().info('Switched to AUTONOMOUS A* Path-Following Mode.')
            return
        if cmd == 'enemy_toggle':
            self._match_robots_enabled = not self._match_robots_enabled
            self.get_logger().info(f'Match Robots (OUR TR, ENEMY BR, ENEMY TR) enabled = {self._match_robots_enabled}')
            return
        if cmd == 'enemy_pause':
            self._match_robots_moving = not self._match_robots_moving
            self.get_logger().info(f'Match Robots moving = {self._match_robots_moving}')
            return

        if cmd == 'ground' or (cmd == 'climb_down' and self.z <= 0.62):
            self._auto_mode = False
            self._start_wheel_lift_climb(-4.35, -5.15, 0.00)
            self.yaw = 1.5707963
            self.get_logger().info('Wheel-Lift Mechanism: Returning to Ground Start/Retry Zone (z=0.00m).')
        elif cmd == 'level_1' or (cmd == 'climb_up' and self.z < 0.35) or (cmd == 'climb_down' and self.z > 0.70):
            # Climb onto Level 1 (+0.60 m)
            tx = self.x if (-2.60 <= self.x <= -0.40) else -2.20
            ty = self.y if (-2.60 <= self.y <= 2.60) else -1.30
            self._start_wheel_lift_climb(tx, ty, 0.60)
            self.get_logger().info(f'Wheel-Lift Mechanism: Climbing to Level 1 ({tx:.2f}, {ty:.2f}, z=0.60m).')
        elif cmd == 'level_2' or (cmd == 'climb_up' and self.z >= 0.35):
            # Climb +300 mm vertical step onto Level 2 (+0.90 m)
            tx = max(-1.10, min(1.10, self.x * 0.55))
            ty = max(-1.10, min(1.10, self.y * 0.55))
            if abs(tx) < 0.45 and abs(ty) < 0.45:
                tx, ty = -0.85, -0.85
            self._start_wheel_lift_climb(tx, ty, 0.90)
            self.get_logger().info(f'Wheel-Lift Mechanism: Climbing +300mm step onto Level 2 ({tx:.2f}, {ty:.2f}, z=0.90m).')
        elif cmd == 'l1_retry':
            self._start_wheel_lift_climb(-1.85, 2.45, 0.60)
            self.yaw = 0.0
            self.get_logger().info('Moved BR to Red Level 1 Retry Zone (-1.85, 2.45, z=0.60m).')
        elif cmd == 'mission_reset':
            self._climb_active = False
            self._auto_mode = True
            self._waypoints = []
            self._wp_index = 0
            self.x, self.y, self.z = -4.35, -5.15, 0.00
            self.pitch = 0.0
            self.yaw = 1.5707963
            self.vx = self.vy = self.wz = 0.0
            self._zone_label = 'GROUND START ZONE (Red)'
            self._gz_odom_ignore_until = time.monotonic() + 0.5
            self._sync_gz_robot_pose(self.x, self.y, self.z, 0.0, self.yaw)
            self.get_logger().info('Mission Reset: BR placed at Ground Start Zone (-4.35, -5.15, z=0.00m) in AUTO mode.')
        elif cmd == 'mission_climb_l1':
            self._start_wheel_lift_climb(-2.50, -1.30, 0.60)
            self.get_logger().info('Mission Wheel-Lift Climb: Ground -> Level 1 Transfer Staging (-2.50, -1.30, z=0.60m).')
        elif cmd == 'mission_climb_l2':
            self._start_wheel_lift_climb(-0.95, 0.00, 0.90)
            self.get_logger().info('Mission Wheel-Lift Climb: Level 1 -> Level 2 Shared Area (-0.95, 0.00, z=0.90m).')

    def _tr_override_cb(self, msg: PointStamped) -> None:
        if msg.header.frame_id.strip().lower() == 'release':
            self._tr_override_active = False
            return
        self._tr_override_active = True
        self._match_robots_enabled = True
        self._our_tr_pose = (float(msg.point.x), float(msg.point.y), float(msg.point.z))

    def _path_cb(self, msg: Path) -> None:
        if not msg.poses:
            # Empty path broadcast means corridor is blocked -> immediate autonomous emergency stop!
            self._waypoints = []
            self._wp_index = 0
            if self._auto_mode:
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                self._safety_brake_active = True
            return
        self._safety_brake_active = False
        self._waypoints = [
            (float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses
        ]
        # When the user clicks a 2D Goal Pose on L1 while on Ground, auto-lift onto L1
        # if in autonomous mode!
        if self._auto_mode and self.z < 0.30 and msg.poses[0].pose.position.z >= 0.45:
            self._start_wheel_lift_climb(-3.45, -1.30, 0.60)

        self._wp_index = 0
        while (
            self._wp_index + 1 < len(self._waypoints)
            and math.hypot(
                self._waypoints[self._wp_index][0] - self.x,
                self._waypoints[self._wp_index][1] - self.y,
            )
            < 0.12
        ):
            self._wp_index += 1

    def _cmd_cb(self, msg: Twist) -> None:
        wheels = mecanum_inverse_kinematics(
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.angular.z),
        )
        self.vx, self.vy, self.wz = mecanum_forward_kinematics(*wheels)

    def _sync_gz_robot_pose(self, x: float, y: float, z: float, pitch: float, yaw: float) -> None:
        """Non-blocking pose teleport to Gazebo Harmonic `br_robot` during/after Wheel-Lift Climb."""
        if not GZ_POSE_SYNC_AVAILABLE or self._gz_node is None or not self._external_gz_odom_received:
            return
        qx, qy, qz, qw = euler_to_quaternion(0.0, pitch, yaw)
        pose = gz_pose_pb2.Pose()
        pose.name = 'br_robot'
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z + 0.02)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)

        def _send() -> None:
            try:
                self._gz_node.request(
                    '/world/robocon_2027_l1_sandbox/set_pose',
                    pose,
                    gz_pose_pb2.Pose,
                    gz_boolean_pb2.Boolean,
                    150,
                )
            except Exception:
                pass

        threading.Thread(target=_send, daemon=True).start()

    def _gz_odom_cb(self, msg: Odometry) -> None:
        self._external_gz_odom_received = True
        if self._climb_active or self._auto_mode or time.monotonic() < self._gz_odom_ignore_until:
            return
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.z = float(msg.pose.pose.position.z)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def _update_wheel_lift_state_machine(self, now_sec: float) -> bool:
        """
        Animate the 3-stage Fusion 360 wheel-lifting step-climbing transition:
          * Stage 1 (0..33%): Front Guide_Front & Caster_Front_L/R lift to upper ledge
          * Stage 2 (33..66%): Dual vertical towers jack chassis & transfer onto inner rollers
          * Stage 3 (66..100%): Rear Caster_Rear_L/R & Guide_Rear retract & lock onto platform
        """
        if not self._climb_active:
            return False

        elapsed = now_sec - self._climb_start_time
        u = min(1.0, max(0.0, elapsed / self._climb_duration))
        sx, sy, sz = self._climb_start_pose
        tx, ty, tz = self._climb_target_pose
        mono = time.monotonic()

        if u < 0.34:
            # Stage 1: Front guides & front wheel legs lift, slight pitch-up
            s1 = u / 0.34
            self.x = sx + 0.25 * (tx - sx) * s1
            self.y = sy + 0.25 * (ty - sy) * s1
            self.z = sz + 0.55 * (tz - sz) * s1
            self.pitch = -0.14 * math.sin(math.pi * s1) if tz > sz else 0.10 * math.sin(math.pi * s1)
            self._climb_stage_text = 'STAGE 1/3: Front Guide & Caster_Front Lift'
        elif u < 0.68:
            # Stage 2: Dual vertical lift columns jack main deck & advance onto inner support rollers
            s2 = (u - 0.34) / 0.34
            self.x = sx + (0.25 + 0.45 * s2) * (tx - sx)
            self.y = sy + (0.25 + 0.45 * s2) * (ty - sy)
            self.z = sz + (0.55 + 0.45 * s2) * (tz - sz)
            self.pitch = -0.06 * (1.0 - s2)
            self._climb_stage_text = 'STAGE 2/3: Dual-Tower Chassis Jack & Inner Roller Transfer'
        elif u < 1.0:
            # Stage 3: Rear Caster_Rear & Guide_Rear retract upward and roll fully onto upper level
            s3 = (u - 0.68) / 0.32
            self.x = sx + (0.70 + 0.30 * s3) * (tx - sx)
            self.y = sy + (0.70 + 0.30 * s3) * (ty - sy)
            self.z = tz
            self.pitch = 0.0
            self._climb_stage_text = 'STAGE 3/3: Rear Caster Retract & Platform Lock'
        else:
            self.x, self.y, self.z = tx, ty, tz
            self.pitch = 0.0
            self._climb_active = False
            self._climb_stage_text = 'IDLE (4 Wheels Locked)'
            self._gz_odom_ignore_until = mono + 0.45
            self._sync_gz_robot_pose(self.x, self.y, self.z, 0.0, self.yaw)
            return True

        if mono - self._gz_last_pose_sync >= 0.15:
            self._gz_last_pose_sync = mono
            self._sync_gz_robot_pose(self.x, self.y, self.z, self.pitch, self.yaw)

        return True

    def _active_match_robot_boxes(self, ref_z: float) -> List[Segment2D]:
        """Return 2D bounding boxes of OUR TR, ENEMY BR, and ENEMY TR within +-0.35 m of ref_z."""
        if not self._match_robots_enabled:
            return []
        segs: List[Segment2D] = []
        for rx, ry, rz in (self._our_tr_pose, self._enemy_br_pose, self._enemy_tr_pose):
            if abs(rz - ref_z) <= 0.35:
                segs.extend(_box_segments(rx, ry, 0.58, 0.58))
        return segs

    def _active_collision_segments(self) -> List[Segment2D]:
        """Return solid obstacle/wall segments on the robot's current elevation for collision sliding."""
        # Allow driving cleanly onto the Ramp & Stairs corridor from Ground/L1
        if 2.95 <= abs(self.x) <= 4.05 and -2.75 <= self.y <= 2.75:
            segs = [
                (-4.00, -2.70, -4.00, 2.70),
                (4.00, -2.70, 4.00, 2.70),
            ]
            return segs

        if self.z < 0.28:
            segs = list(self._ground_segments)
            segs = [
                s for s in segs
                if s not in (
                    (-4.00, -2.70, -3.00, -2.70),
                    (-4.00, 2.70, -3.00, 2.70),
                    (3.00, -2.70, 4.00, -2.70),
                    (3.00, 2.70, 4.00, 2.70),
                )
            ]
            segs.extend(self._active_match_robot_boxes(self.z))
            return segs
        if self.z >= 0.78:
            segs = list(self._l2_segments)
            segs.extend([
                (-1.50, -1.50, 1.50, -1.50),
                (1.50, -1.50, 1.50, 1.50),
                (1.50, 1.50, -1.50, 1.50),
                (-1.50, 1.50, -1.50, -1.50),
            ])
            segs.extend(self._active_match_robot_boxes(self.z))
            return segs

        segs = list(self._l1_segments)
        segs.extend(self._active_match_robot_boxes(self.z))
        return segs

    def _directional_lidar_clearance(self, move_dir_body_rad: float, half_cone_rad: float = 0.785) -> float:
        """Return minimum LiDAR range within +-half_cone_rad of the robot's commanded motion direction."""
        if self._last_scan_meta is None:
            return self._min_lidar_dist
        angle_min, angle_inc, ranges = self._last_scan_meta
        min_r = 10.0
        for i, r in enumerate(ranges):
            beam_ang = angle_min + i * angle_inc
            if abs(wrap_angle(beam_ang - move_dir_body_rad)) <= half_cone_rad:
                if r < min_r:
                    min_r = r
        return min_r

    def _update_path_follower(self) -> None:
        """
        Decoupled holonomic mecanum path follower with proportional yaw alignment
        and directional forward-cone LiDAR emergency safety brake.
        """
        if not self._auto_mode or not self._waypoints or self._climb_active:
            return
        if self._wp_index >= len(self._waypoints):
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
            return

        tx, ty = self._waypoints[self._wp_index]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        while dist < 0.12 and self._wp_index + 1 < len(self._waypoints):
            self._wp_index += 1
            tx, ty = self._waypoints[self._wp_index]
            dx = tx - self.x
            dy = ty - self.y
            dist = math.hypot(dx, dy)

        if dist < 0.05 and self._wp_index == len(self._waypoints) - 1:
            self._wp_index = len(self._waypoints)
            self._safety_brake_active = False
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
            return

        # Directional LiDAR safety shield: brake ONLY if an obstacle is <0.38m ahead along motion vector
        move_dir_body = wrap_angle(math.atan2(dy, dx) - self.yaw)
        fwd_clearance = self._directional_lidar_clearance(move_dir_body, half_cone_rad=0.72)
        if fwd_clearance < 0.38:
            self._safety_brake_active = True
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
            return
        self._safety_brake_active = False

        speed = min(float(self.get_parameter('follow_speed_mps').value), max(0.15, dist * 2.5))
        world_vx = speed * (dx / max(1e-6, dist))
        world_vy = speed * (dy / max(1e-6, dist))

        # Decoupled proportional yaw alignment toward lookahead waypoint
        target_yaw = math.atan2(dy, dx)
        yaw_err = wrap_angle(target_yaw - self.yaw)
        desired_wz = max(-1.2, min(1.2, 2.0 * yaw_err))

        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        body_vx = world_vx * cos_y + world_vy * sin_y
        body_vy = -world_vx * sin_y + world_vy * cos_y

        wheels = mecanum_inverse_kinematics(body_vx, body_vy, desired_wz)
        self.vx, self.vy, self.wz = mecanum_forward_kinematics(*wheels)

    def _step_kinematics_and_tf(self) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        if not self._update_wheel_lift_state_machine(now_sec):
            self._update_path_follower()

            if (not self._external_gz_odom_received or self._auto_mode) and 0.0 < dt < 1.0:
                cos_y = math.cos(self.yaw)
                sin_y = math.sin(self.yaw)
                cand_x = max(-5.14, min(5.14, self.x + (self.vx * cos_y - self.vy * sin_y) * dt))
                cand_y = max(-5.14, min(5.14, self.y + (self.vx * sin_y + self.vy * cos_y) * dt))
                self.yaw = wrap_angle(self.yaw + self.wz * dt)

                # Axis-separated sliding collision physics against active level walls & enemy robot
                coll_segs = self._active_collision_segments()
                r_col = self._collision_radius
                if not is_circle_colliding(cand_x, cand_y, r_col, coll_segs):
                    self.x, self.y = cand_x, cand_y
                elif not is_circle_colliding(cand_x, self.y, r_col, coll_segs):
                    self.x = cand_x
                elif not is_circle_colliding(self.x, cand_y, r_col, coll_segs):
                    self.y = cand_y
                elif is_circle_colliding(self.x, self.y, r_col, coll_segs):
                    # Allow escaping slight wall overlap only when strictly increasing minimum clearance
                    # and never moving deeper into static arena walls!
                    static_only = self._l1_segments if (0.28 <= self.z < 0.78) else (
                        self._l2_segments if self.z >= 0.78 else self._ground_segments
                    )
                    if not is_circle_colliding(cand_x, cand_y, r_col, static_only):
                        old_min = min((point_to_segment_distance(self.x, self.y, s) for s in coll_segs), default=10.0)
                        new_min = min((point_to_segment_distance(cand_x, cand_y, s) for s in coll_segs), default=10.0)
                        if new_min > old_min:
                            self.x, self.y = cand_x, cand_y

                # Update terrain elevation z & pitch smoothly on Ramps, Stairs, L1, L2
                self.z, self.pitch, self._zone_label = compute_terrain_elevation(
                    self.x, self.y, self.z
                )

                if self._auto_mode and self._external_gz_odom_received:
                    mono = time.monotonic()
                    if mono - self._gz_last_pose_sync >= 0.12:
                        self._gz_last_pose_sync = mono
                        self._sync_gz_robot_pose(self.x, self.y, self.z, self.pitch, self.yaw)

        stamp = now.to_msg()
        odom_frame = str(self.get_parameter('odom_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)

        qx, qy, qz, qw = euler_to_quaternion(0.0, self.pitch, self.yaw)

        # Broadcast odom -> base_link
        tf_odom_base = TransformStamped()
        tf_odom_base.header.stamp = stamp
        tf_odom_base.header.frame_id = odom_frame
        tf_odom_base.child_frame_id = base_frame
        tf_odom_base.transform.translation.x = self.x
        tf_odom_base.transform.translation.y = self.y
        tf_odom_base.transform.translation.z = self.z
        tf_odom_base.transform.rotation.x = qx
        tf_odom_base.transform.rotation.y = qy
        tf_odom_base.transform.rotation.z = qz
        tf_odom_base.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_odom_base)

        if bool(self.get_parameter('publish_internal_odom').value):
            odom_msg = Odometry()
            odom_msg.header.stamp = stamp
            odom_msg.header.frame_id = odom_frame
            odom_msg.child_frame_id = base_frame
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = self.z
            odom_msg.pose.pose.orientation.x = qx
            odom_msg.pose.pose.orientation.y = qy
            odom_msg.pose.pose.orientation.z = qz
            odom_msg.pose.pose.orientation.w = qw
            odom_msg.twist.twist.linear.x = self.vx
            odom_msg.twist.twist.linear.y = self.vy
            odom_msg.twist.twist.angular.z = self.wz
            self.odom_pub.publish(odom_msg)

        # Update 3 Match Robots (OUR TR, ENEMY BR, ENEMY TR) along their Rulebook-legal trajectories
        if self._match_robots_enabled and self._match_robots_moving and 0.0 < dt < 1.0:
            self._match_time_s += dt
            if not self._tr_override_active:
                self._our_tr_pose = interpolate_closed_waypoints(
                    self._match_waypoints['our_tr'], self._match_time_s, speed_mps=0.95
                )
            self._enemy_br_pose = interpolate_closed_waypoints(
                self._match_waypoints['enemy_br'], self._match_time_s + 4.5, speed_mps=0.85
            )
            self._enemy_tr_pose = interpolate_closed_waypoints(
                self._match_waypoints['enemy_tr'], self._match_time_s + 2.0, speed_mps=0.95
            )
        self._publish_enemy_markers(stamp)

        mode_str = 'AUTO (A* Follower)' if self._auto_mode else 'MANUAL (Joystick/WASD)'
        match_str = 'ON' if self._match_robots_enabled else 'OFF'
        brake_str = ' | [SAFETY BRAKE!]' if self._safety_brake_active else ''
        status_msg = String()
        status_msg.data = (
            f'Mode: {mode_str} | Zone: {self._zone_label} | '
            f'Pose: ({self.x:.2f}, {self.y:.2f}, z={self.z:.2f}m) | '
            f'MinLiDAR: {self._min_lidar_dist:.2f}m | Wheel-Lift: {self._climb_stage_text} | '
            f'MatchRobots(TR/Enemy): {match_str}{brake_str}'
        )
        self.status_pub.publish(status_msg)

    def _publish_enemy_markers(self, stamp) -> None:
        """
        Publish 3D MarkerArray for the 3 autonomous match robots (Rule 11.1):
          * OUR TR   (Red Team Transporter Robot — Ground + Red Ramp + Red Transfer Area)
          * ENEMY BR (Blue Team Builder Robot    — Blue Transfer + Blue L1 + L1/L2 Shared Areas)
          * ENEMY TR (Blue Team Transporter Robot — Blue Ground + Ground Shared + Blue Ramp + Blue Transfer)
        """
        map_frame = str(self.get_parameter('map_frame').value)
        m_array = MarkerArray()
        robots = (
            (0, 'OUR TR (Red)', self._our_tr_pose, (0.88, 0.16, 0.16), (1.0, 0.80, 0.10), 0.52),
            (1, 'ENEMY BR (Blue)', self._enemy_br_pose, (0.12, 0.25, 0.92), (1.0, 0.35, 0.05), 0.62),
            (2, 'ENEMY TR (Blue)', self._enemy_tr_pose, (0.15, 0.60, 0.95), (1.0, 0.85, 0.15), 0.52),
        )
        for idx, label, (rx, ry, rz), body_rgb, beacon_rgb, height_m in robots:
            body = Marker()
            body.header.stamp = stamp
            body.header.frame_id = map_frame
            body.ns = 'enemy_robots'
            body.id = idx * 3
            body.type = Marker.CUBE
            body.action = Marker.ADD if self._match_robots_enabled else Marker.DELETE
            body.pose.position.x = float(rx)
            body.pose.position.y = float(ry)
            body.pose.position.z = float(rz + height_m * 0.5)
            body.pose.orientation.w = 1.0
            body.scale.x = 0.58
            body.scale.y = 0.58
            body.scale.z = float(height_m)
            body.color.r = float(body_rgb[0])
            body.color.g = float(body_rgb[1])
            body.color.b = float(body_rgb[2])
            body.color.a = 0.95
            m_array.markers.append(body)

            beacon = Marker()
            beacon.header.stamp = stamp
            beacon.header.frame_id = map_frame
            beacon.ns = 'enemy_robots'
            beacon.id = idx * 3 + 1
            beacon.type = Marker.CYLINDER
            beacon.action = Marker.ADD if self._match_robots_enabled else Marker.DELETE
            beacon.pose.position.x = float(rx)
            beacon.pose.position.y = float(ry)
            beacon.pose.position.z = float(rz + height_m + 0.07)
            beacon.pose.orientation.w = 1.0
            beacon.scale.x = 0.18
            beacon.scale.y = 0.18
            beacon.scale.z = 0.14
            beacon.color.r = float(beacon_rgb[0])
            beacon.color.g = float(beacon_rgb[1])
            beacon.color.b = float(beacon_rgb[2])
            beacon.color.a = 1.0
            m_array.markers.append(beacon)

            txt = Marker()
            txt.header.stamp = stamp
            txt.header.frame_id = map_frame
            txt.ns = 'enemy_robots'
            txt.id = idx * 3 + 2
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD if self._match_robots_enabled else Marker.DELETE
            txt.pose.position.x = float(rx)
            txt.pose.position.y = float(ry)
            txt.pose.position.z = float(rz + height_m + 0.34)
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.22
            txt.color.r = 1.0
            txt.color.g = 0.92
            txt.color.b = 0.25
            txt.color.a = 1.0
            txt.text = label
            m_array.markers.append(txt)

        self.enemy_pub.publish(m_array)

    def _publish_scan(self) -> None:
        if not bool(self.get_parameter('publish_internal_scan').value):
            return

        num_beams = int(self.get_parameter('num_beams').value)
        r_min = float(self.get_parameter('range_min').value)
        r_max = float(self.get_parameter('range_max').value)

        if self.z < 0.28:
            active_segs = list(self._ground_segments)
        elif self.z >= 0.78:
            active_segs = list(self._l2_segments)
        else:
            active_segs = list(self._l1_segments)

        # Inject any of the 3 match robots (OUR TR, ENEMY BR, ENEMY TR) currently on the same floor
        active_segs.extend(self._active_match_robot_boxes(self.z))

        angle_min, angle_inc, ranges = simulate_l1_laser_scan(
            sensor_x=self.x,
            sensor_y=self.y,
            sensor_yaw=self.yaw,
            num_beams=num_beams,
            range_min=r_min,
            range_max=r_max,
            segments=active_segs,
        )
        self._min_lidar_dist = min(ranges) if ranges else r_max
        self._last_scan_meta = (angle_min, angle_inc, ranges)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = str(self.get_parameter('lidar_frame').value)
        scan.angle_min = angle_min
        scan.angle_max = angle_min + (num_beams - 1) * angle_inc
        scan.angle_increment = angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 0.10
        scan.range_min = r_min
        scan.range_max = r_max
        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = L1SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
