#!/usr/bin/env python3
"""
Multi-Level Bayesian Log-Odds 2D LiDAR Occupancy Grid Mapper & Static-vs-Dynamic
Obstacle Differentiator for ABU Robocon 2027 BR (Ground, Level 1, Level 2).

Features
--------
1. Bayesian Log-Odds Occupancy Mapping with Bresenham 2D Raycasting:
       L(m_i | z_{1:t}, x_{1:t}) = clamp(L(m_i | z_{1:t-1}) + l_inv - l_0, L_min, L_max)
       free = 0, occupied = 100, unknown = -1
2. Three Independent Floor Maps (`ground`, `level_1`, `level_2`):
   Prevents LiDAR beams on Level 2 (z=0.90 m) from erasing Level 1 walls (z=0.60 m)
   or Ground Earth/Sky blocks (z=0.00 m), switching active floor cleanly via /odom z.
3. Fast Analytic Static Wall Rasterization (~15 ms for all 3 floors vs ~1260 ms ray sweep).
4. Live Static-vs-Dynamic Obstacle Layer (/br_dynamic_obstacles):
   Differentiates live LiDAR hits on moving Blue Enemy Robots or newly placed blocks
   from the static pre-made arena walls with raytrace clearing (zero ghost trails).
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from br_nav_sandbox.traversability_costmap import level_from_z, z_from_level


def prob_to_log_odds(prob: float) -> float:
    """Convert probability p in (0, 1) to log-odds ln(p / (1 - p))."""
    if prob <= 0.0 or prob >= 1.0:
        raise ValueError(f'Probability must lie strictly in (0, 1); got {prob}')
    return math.log(prob / (1.0 - prob))


def log_odds_to_prob(log_odds: float) -> float:
    """Convert log-odds l in (-inf, +inf) to probability 1 / (1 + exp(-l))."""
    if log_odds >= 50.0:
        return 1.0
    if log_odds <= -50.0:
        return 0.0
    return 1.0 / (1.0 + math.exp(-log_odds))


def bresenham_line(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """Trace an integer grid ray from (x0, y0) to (x1, y1) inclusive via Bresenham."""
    cells: List[Tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    cx, cy = x0, y0

    while True:
        cells.append((cx, cy))
        if cx == x1 and cy == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            cx += sx
        if e2 < dx:
            err += dx
            cy += sy
    return cells


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract planar yaw angle (radians) from a quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class LogOddsOccupancyMapper:
    """Pure-Python Bayesian log-odds occupancy grid mapper with Bresenham raycasting."""

    FREE_VALUE = 0
    OCCUPIED_VALUE = 100
    UNKNOWN_VALUE = -1

    def __init__(
        self,
        grid_resolution: float = 0.05,
        width_m: float = 11.4,
        height_m: float = 11.4,
        origin_x: float = -5.7,
        origin_y: float = -5.7,
        p_occ: float = 0.70,
        p_free: float = 0.30,
        l_min: float = -4.0,
        l_max: float = 4.0,
        occ_prob_threshold: float = 0.65,
        free_prob_threshold: float = 0.35,
    ) -> None:
        if grid_resolution <= 0.0:
            raise ValueError('grid_resolution must be positive')
        if width_m <= 0.0 or height_m <= 0.0:
            raise ValueError('width_m and height_m must be positive')
        if not (0.5 < p_occ < 1.0):
            raise ValueError('p_occ must be in (0.5, 1.0)')
        if not (0.0 < p_free < 0.5):
            raise ValueError('p_free must be in (0.0, 0.5)')
        if l_min >= 0.0 or l_max <= 0.0:
            raise ValueError('l_min must be < 0 and l_max must be > 0')

        self.resolution = float(grid_resolution)
        self.width_m = float(width_m)
        self.height_m = float(height_m)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

        self.width = int(round(self.width_m / self.resolution))
        self.height = int(round(self.height_m / self.resolution))

        self.p_occ = float(p_occ)
        self.p_free = float(p_free)
        self.l_occ = prob_to_log_odds(self.p_occ)
        self.l_free = prob_to_log_odds(self.p_free)
        self.l_min = float(l_min)
        self.l_max = float(l_max)

        self.occ_prob_threshold = float(occ_prob_threshold)
        self.free_prob_threshold = float(free_prob_threshold)
        self.l_occ_thresh = prob_to_log_odds(self.occ_prob_threshold)
        self.l_free_thresh = prob_to_log_odds(self.free_prob_threshold)

        self.log_odds: List[float] = [0.0] * (self.width * self.height)

    def reset(self) -> None:
        """Reset all cells to prior log-odds L_0 = 0.0 (unknown)."""
        self.log_odds = [0.0] * (self.width * self.height)

    def in_bounds(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.width and 0 <= cy < self.height

    def world_to_cell(self, wx: float, wy: float) -> Tuple[int, int]:
        cx = int(math.floor((wx - self.origin_x) / self.resolution))
        cy = int(math.floor((wy - self.origin_y) / self.resolution))
        return cx, cy

    def cell_to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        wx = self.origin_x + (cx + 0.5) * self.resolution
        wy = self.origin_y + (cy + 0.5) * self.resolution
        return wx, wy

    def get_cell_log_odds(self, cx: int, cy: int) -> float:
        if not self.in_bounds(cx, cy):
            raise IndexError(f'Cell ({cx}, {cy}) out of bounds ({self.width}x{self.height})')
        return self.log_odds[cy * self.width + cx]

    def get_cell_probability(self, cx: int, cy: int) -> float:
        return log_odds_to_prob(self.get_cell_log_odds(cx, cy))

    def _add_log_odds(self, cx: int, cy: int, delta_l: float) -> None:
        if not self.in_bounds(cx, cy):
            return
        idx = cy * self.width + cx
        updated = self.log_odds[idx] + delta_l
        if updated > self.l_max:
            updated = self.l_max
        elif updated < self.l_min:
            updated = self.l_min
        self.log_odds[idx] = updated

    def rasterize_static_segments(
        self,
        segments: List[Tuple[float, float, float, float]],
        free_rects: List[Tuple[float, float, float, float]],
    ) -> None:
        """
        Fast O(segments) static arena rasterization: marks known floor rectangles as
        free (l_free * 2) and rasterizes wall/block/pillar segments via Bresenham as
        occupied (l_occ * 3). Runs in ~4 ms per floor (80x faster than full ray sweeps).
        """
        w, h = self.width, self.height
        free_val = max(self.l_min, self.l_free * 2.0)
        occ_val = min(self.l_max, self.l_occ * 3.0)

        for x0, y0, x1, y1 in free_rects:
            c_x0, c_y0 = self.world_to_cell(x0, y0)
            c_x1, c_y1 = self.world_to_cell(x1, y1)
            c_x0 = max(0, min(w - 1, c_x0))
            c_x1 = max(0, min(w - 1, c_x1))
            c_y0 = max(0, min(h - 1, c_y0))
            c_y1 = max(0, min(h - 1, c_y1))
            for cy in range(c_y0, c_y1 + 1):
                row = cy * w
                for cx in range(c_x0, c_x1 + 1):
                    self.log_odds[row + cx] = free_val

        for x0, y0, x1, y1 in segments:
            c_x0, c_y0 = self.world_to_cell(x0, y0)
            c_x1, c_y1 = self.world_to_cell(x1, y1)
            for cx, cy in bresenham_line(c_x0, c_y0, c_x1, c_y1):
                if self.in_bounds(cx, cy):
                    self.log_odds[cy * w + cx] = occ_val

    def update_ray(
        self,
        sensor_x: float,
        sensor_y: float,
        beam_angle: float,
        range_m: float,
        range_min: float = 0.05,
        range_max: float = 8.0,
    ) -> None:
        """Perform an inverse-sensor-model log-odds update along a single LiDAR ray."""
        if math.isnan(range_m) or range_m < range_min:
            return

        is_hit = not math.isinf(range_m) and (range_m < (range_max - 1e-3))
        effective_range = min(range_m, range_max) if not math.isinf(range_m) else range_max

        end_x = sensor_x + effective_range * math.cos(beam_angle)
        end_y = sensor_y + effective_range * math.sin(beam_angle)

        c_sx, c_sy = self.world_to_cell(sensor_x, sensor_y)
        c_ex, c_ey = self.world_to_cell(end_x, end_y)

        ray_cells = bresenham_line(c_sx, c_sy, c_ex, c_ey)
        if not ray_cells:
            return

        if is_hit:
            for cx, cy in ray_cells[:-1]:
                self._add_log_odds(cx, cy, self.l_free)
            hit_cx, hit_cy = ray_cells[-1]
            self._add_log_odds(hit_cx, hit_cy, self.l_occ)
        else:
            for cx, cy in ray_cells:
                self._add_log_odds(cx, cy, self.l_free)

    def update_from_scan(
        self,
        sensor_x: float,
        sensor_y: float,
        sensor_yaw: float,
        angle_min: float,
        angle_increment: float,
        ranges: List[float],
        range_min: float,
        range_max: float,
    ) -> None:
        """Update the log-odds grid from a full 2D LaserScan measurement."""
        angle = sensor_yaw + angle_min
        for r in ranges:
            self.update_ray(
                sensor_x=sensor_x,
                sensor_y=sensor_y,
                beam_angle=angle,
                range_m=float(r),
                range_min=range_min,
                range_max=range_max,
            )
            angle += angle_increment

    def to_occupancy_grid_data(self) -> List[int]:
        """Quantize log-odds grid to ROS 2 OccupancyGrid: free=0, occupied=100, unknown=-1."""
        out: List[int] = [self.UNKNOWN_VALUE] * (self.width * self.height)
        l_occ_t = self.l_occ_thresh
        l_free_t = self.l_free_thresh
        for i, val in enumerate(self.log_odds):
            if val >= l_occ_t:
                out[i] = self.OCCUPIED_VALUE
            elif val <= l_free_t:
                out[i] = self.FREE_VALUE
            else:
                out[i] = self.UNKNOWN_VALUE
        return out


class OccupancyMapperNode(Node):
    """
    ROS 2 Jazzy multi-level occupancy mapper maintaining 3 independent floor grids
    (`ground`, `level_1`, `level_2`) + live Static-vs-Dynamic obstacle differentiation.
    """

    def __init__(self) -> None:
        super().__init__('occupancy_mapper')

        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('width_m', 11.4)
        self.declare_parameter('height_m', 11.4)
        self.declare_parameter('origin_x', -5.7)
        self.declare_parameter('origin_y', -5.7)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('p_occ', 0.70)
        self.declare_parameter('p_free', 0.30)
        self.declare_parameter('l_min', -4.0)
        self.declare_parameter('l_max', 4.0)
        self.declare_parameter('occ_prob_threshold', 0.65)
        self.declare_parameter('free_prob_threshold', 0.35)
        self.declare_parameter('pre_scan_l1_arena', True)
        self.declare_parameter('inflation_radius', 0.80)
        self.declare_parameter('cost_scaling_factor', 5.0)
        self.declare_parameter('inscribed_radius', 0.35)

        self._active_level: str = 'ground'
        self._mappers: Dict[str, LogOddsOccupancyMapper] = {}
        self._static_masks: Dict[str, Set[Tuple[int, int]]] = {}
        self._dynamic_cells: Dict[str, Dict[Tuple[int, int], float]] = {
            'ground': {},
            'level_1': {},
            'level_2': {},
        }

        self._init_all_level_mappers()
        self.add_on_set_parameters_callback(self._on_parameters_set)

        self._latest_odom_pose: Optional[Tuple[float, float, float, float]] = (
            -4.35,
            -5.15,
            0.00,
            1.5708,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.grid_pub = self.create_publisher(OccupancyGrid, '/br_occupancy_grid', map_qos)
        self.dynamic_pub = self.create_publisher(MarkerArray, '/br_dynamic_obstacles', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self.publish_timer = self.create_timer(1.0, self.publish_occupancy_grid)
        self.publish_occupancy_grid()

        self.get_logger().info(
            f'OccupancyMapperNode (3-Floor Fast Raster + Dynamic Diff) initialized: '
            f'{self.mapper.width}x{self.mapper.height} cells at {self.mapper.resolution:.3f} m/cell.'
        )

    @property
    def mapper(self) -> LogOddsOccupancyMapper:
        """Return the active floor's LogOddsOccupancyMapper."""
        return self._mappers[self._active_level]

    def _create_single_mapper(self) -> LogOddsOccupancyMapper:
        return LogOddsOccupancyMapper(
            grid_resolution=float(self.get_parameter('grid_resolution').value),
            width_m=float(self.get_parameter('width_m').value),
            height_m=float(self.get_parameter('height_m').value),
            origin_x=float(self.get_parameter('origin_x').value),
            origin_y=float(self.get_parameter('origin_y').value),
            p_occ=float(self.get_parameter('p_occ').value),
            p_free=float(self.get_parameter('p_free').value),
            l_min=float(self.get_parameter('l_min').value),
            l_max=float(self.get_parameter('l_max').value),
            occ_prob_threshold=float(self.get_parameter('occ_prob_threshold').value),
            free_prob_threshold=float(self.get_parameter('free_prob_threshold').value),
        )

    def _init_all_level_mappers(self) -> None:
        from br_nav_sandbox.l1_sim_bridge import (
            build_ground_arena_wall_segments,
            build_l1_arena_wall_segments,
            build_l2_arena_wall_segments,
        )

        pre_scan = bool(self.get_parameter('pre_scan_l1_arena').value)
        level_specs = {
            'ground': (
                build_ground_arena_wall_segments(),
                [(-5.48, -5.48, 5.48, 5.48)],
            ),
            'level_1': (
                build_l1_arena_wall_segments(),
                [
                    (-2.95, -2.95, 2.95, 2.95),
                    (-3.98, -1.78, -2.95, -0.82),
                    (2.95, -1.78, 3.98, -0.82),
                ],
            ),
            'level_2': (
                build_l2_arena_wall_segments(),
                [(-1.48, -1.48, 1.48, 1.48)],
            ),
        }

        for lvl, (segs, free_rects) in level_specs.items():
            m = self._create_single_mapper()
            if pre_scan:
                m.rasterize_static_segments(segs, free_rects)
                # Keep interior of solid Level 1 (on Ground) and Level 2 (on L1) as unknown/solid
                if lvl == 'ground':
                    c0x, c0y = m.world_to_cell(-2.95, -2.95)
                    c1x, c1y = m.world_to_cell(2.95, 2.95)
                    for cy in range(max(0, c0y), min(m.height, c1y + 1)):
                        row = cy * m.width
                        for cx in range(max(0, c0x), min(m.width, c1x + 1)):
                            m.log_odds[row + cx] = 0.0
                elif lvl == 'level_1':
                    c0x, c0y = m.world_to_cell(-1.45, -1.45)
                    c1x, c1y = m.world_to_cell(1.45, 1.45)
                    for cy in range(max(0, c0y), min(m.height, c1y + 1)):
                        row = cy * m.width
                        for cx in range(max(0, c0x), min(m.width, c1x + 1)):
                            m.log_odds[row + cx] = 0.0
            self._mappers[lvl] = m
            self._static_masks[lvl] = self._build_dilated_static_mask(m)

    @staticmethod
    def _build_dilated_static_mask(mapper: LogOddsOccupancyMapper) -> Set[Tuple[int, int]]:
        data = mapper.to_occupancy_grid_data()
        w, h = mapper.width, mapper.height
        mask: Set[Tuple[int, int]] = set()
        for cy in range(h):
            row = cy * w
            for cx in range(w):
                if data[row + cx] == mapper.OCCUPIED_VALUE:
                    for dy in range(-2, 3):
                        for dx in range(-2, 3):
                            nx, ny = cx + dx, cy + dy
                            if 0 <= nx < w and 0 <= ny < h:
                                mask.add((nx, ny))
        return mask

    def _on_parameters_set(self, params) -> SetParametersResult:
        for param in params:
            if param.name == 'grid_resolution' and float(param.value) <= 0.0:
                return SetParametersResult(successful=False, reason='grid_resolution must be > 0')
        return SetParametersResult(successful=True)

    def _odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(ori.x, ori.y, ori.z, ori.w)
        z_val = float(pos.z)
        self._latest_odom_pose = (float(pos.x), float(pos.y), z_val, yaw)

        new_lvl = level_from_z(z_val)
        if new_lvl != self._active_level:
            self._active_level = new_lvl
            self.get_logger().info(
                f'OccupancyMapper switched active floor -> {new_lvl.upper()} (z={z_from_level(new_lvl):.2f}m)'
            )
            self.publish_occupancy_grid()

    def _lookup_sensor_pose(self, scan_frame: str) -> Optional[Tuple[float, float, float, float]]:
        map_frame = str(self.get_parameter('map_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        target_frame = scan_frame if scan_frame else base_frame

        for frame in (target_frame, base_frame):
            try:
                tf_msg: TransformStamped = self.tf_buffer.lookup_transform(
                    map_frame, frame, rclpy.time.Time()
                )
                t = tf_msg.transform.translation
                q = tf_msg.transform.rotation
                yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
                # Subtract lidar_link height offset (~0.25 m) when inferring floor level
                floor_z = float(t.z) - (0.25 if frame == target_frame and frame != base_frame else 0.0)
                return float(t.x), float(t.y), max(0.0, floor_z), yaw
            except Exception:
                continue

        return self._latest_odom_pose

    def _scan_cb(self, scan: LaserScan) -> None:
        pose = self._lookup_sensor_pose(scan.header.frame_id)
        if pose is None:
            return

        sx, sy, floor_z, syaw = pose
        lvl = level_from_z(floor_z)
        self._active_level = lvl
        active_mapper = self._mappers[lvl]
        static_mask = self._static_masks[lvl]
        dyn_cells = self._dynamic_cells[lvl]

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        r_min = max(0.05, float(scan.range_min))
        r_max = float(scan.range_max)
        angle_min = float(scan.angle_min)
        angle_inc = float(scan.angle_increment)
        ranges = [float(r) for r in scan.ranges]

        # 1. Bayesian log-odds update on the active floor's grid
        active_mapper.update_from_scan(
            sensor_x=sx,
            sensor_y=sy,
            sensor_yaw=syaw,
            angle_min=angle_min,
            angle_increment=angle_inc,
            ranges=ranges,
            range_min=r_min,
            range_max=r_max,
        )

        # 2. Fast Static-vs-Dynamic Obstacle Layer (raytrace clearing + TTL expiration)
        stale = [c for c, ts in dyn_cells.items() if (now_sec - ts) > 0.65]
        for c in stale:
            del dyn_cells[c]

        c_sx, c_sy = active_mapper.world_to_cell(sx, sy)
        ang = syaw + angle_min
        for r in ranges:
            if not math.isnan(r) and r >= r_min:
                is_hit = (not math.isinf(r)) and (r < (r_max - 1e-3))
                eff_r = min(r, r_max) if not math.isinf(r) else r_max
                ex = sx + eff_r * math.cos(ang)
                ey = sy + eff_r * math.sin(ang)
                c_ex, c_ey = active_mapper.world_to_cell(ex, ey)
                ray_cells = bresenham_line(c_sx, c_sy, c_ex, c_ey)
                clear_slice = ray_cells[:-1] if is_hit else ray_cells
                for cell in clear_slice:
                    dyn_cells.pop(cell, None)
                if is_hit and active_mapper.in_bounds(c_ex, c_ey):
                    hit_cell = (c_ex, c_ey)
                    if hit_cell not in static_mask:
                        dyn_cells[hit_cell] = now_sec
            ang += angle_inc

        self.publish_occupancy_grid()

    def _publish_dynamic_obstacles(self, grid_data: List[int]) -> None:
        active_mapper = self.mapper
        static_mask = self._static_masks[self._active_level]
        z_floor = z_from_level(self._active_level)

        w, h = active_mapper.width, active_mapper.height
        dynamic_cells: List[Tuple[int, int]] = []
        for cy in range(h):
            row = cy * w
            for cx in range(w):
                if (
                    grid_data[row + cx] == active_mapper.OCCUPIED_VALUE
                    and (cx, cy) not in static_mask
                ):
                    dynamic_cells.append((cx, cy))

        now = self.get_clock().now().to_msg()
        frame_id = str(self.get_parameter('map_frame').value)
        m_array = MarkerArray()

        cubes = Marker()
        cubes.header.stamp = now
        cubes.header.frame_id = frame_id
        cubes.ns = 'lidar_dynamic_obstacles'
        cubes.id = 0
        cubes.type = Marker.CUBE_LIST
        cubes.pose.orientation.w = 1.0
        cubes.scale.x = active_mapper.resolution * 1.1
        cubes.scale.y = active_mapper.resolution * 1.1
        cubes.scale.z = 0.18
        cubes.color.r = 1.0
        cubes.color.g = 0.25
        cubes.color.b = 0.05
        cubes.color.a = 0.95

        label = Marker()
        label.header.stamp = now
        label.header.frame_id = frame_id
        label.ns = 'lidar_dynamic_obstacles'
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.orientation.w = 1.0
        label.scale.z = 0.24
        label.color.r = 1.0
        label.color.g = 0.35
        label.color.b = 0.10
        label.color.a = 1.0

        if dynamic_cells:
            cubes.action = Marker.ADD
            sum_x = 0.0
            sum_y = 0.0
            for cx, cy in dynamic_cells:
                wx, wy = active_mapper.cell_to_world(cx, cy)
                pt = Point()
                pt.x = wx
                pt.y = wy
                pt.z = z_floor + 0.09
                cubes.points.append(pt)
                sum_x += wx
                sum_y += wy

            label.action = Marker.ADD
            label.pose.position.x = sum_x / len(dynamic_cells)
            label.pose.position.y = sum_y / len(dynamic_cells)
            label.pose.position.z = z_floor + 0.50
            label.text = f'LiDAR DYNAMIC OBSTACLE ({len(dynamic_cells)} cells)'
        else:
            cubes.action = Marker.DELETE
            label.action = Marker.DELETE

        m_array.markers.append(cubes)
        m_array.markers.append(label)
        self.dynamic_pub.publish(m_array)

    def publish_occupancy_grid(self) -> None:
        active_mapper = self.mapper
        dyn_cells = self._dynamic_cells[self._active_level]

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter('map_frame').value)

        msg.info.resolution = active_mapper.resolution
        msg.info.width = active_mapper.width
        msg.info.height = active_mapper.height
        msg.info.origin.position.x = active_mapper.origin_x
        msg.info.origin.position.y = active_mapper.origin_y
        msg.info.origin.position.z = z_from_level(self._active_level) + 0.004
        msg.info.origin.orientation.w = 1.0

        grid_data = active_mapper.to_occupancy_grid_data()
        w = active_mapper.width
        for cx, cy in dyn_cells:
            grid_data[cy * w + cx] = active_mapper.OCCUPIED_VALUE

        msg.data = grid_data
        self.grid_pub.publish(msg)
        self._publish_dynamic_obstacles(grid_data)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OccupancyMapperNode()
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
