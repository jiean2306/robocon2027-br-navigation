#!/usr/bin/env python3
"""
High-Speed Vectorized Multi-Level Traversability & Footprint Costmap Generator
for ABU Robocon 2027 BR (Ground, Level 1, and Level 2).

Mathematical Formulation
------------------------
The Builder Robot (BR v7) has footprint L x W ~= 0.70 m x 0.70 m (Rule 11.5):
  * Inscribed radius:     r_inscribed     = 0.35 m
  * Circumscribed radius: r_circumscribed = 0.70 / sqrt(2) ~= 0.495 m (~0.50 m)
  * Inflation radius:     r_inflation     >= r_circumscribed (default 0.80 m)

Exact Euclidean Distance Transform (EDT) yields distance d (meters) from every grid
cell to the nearest lethal cell (wall, barrier, drop-off, restricted zone, or live
LiDAR-detected dynamic obstacle / Enemy Robot):

  1. Lethal obstacle / restricted boundary (d = 0):          C_254 = 254 (COST_LETHAL)
  2. Inscribed footprint radius (0 < d <= r_inscribed):      C_254 = 253 (COST_INSCRIBED)
  3. Exponential decay halo (r_inscribed < d <= r_inflation):
         C_254(d) = floor( 252 * exp(-cost_scaling_factor * (d - r_inscribed)) )
  4. Free space (d > r_inflation):                           C_254 = 0   (COST_FREE)

Encoding in nav_msgs/OccupancyGrid (signed int8, -1..100):
  * 254 -> 100 (RViz2 lethal)
  * 253 ->  99 (RViz2 inscribed cyan contour)
  * 1..252 -> 1..98 (linearly scaled decay)
  * 0   ->   0 (free)
  * 255 ->  -1 (unknown)
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Point, Point32, PolygonStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray


COST_FREE = 0
COST_INSCRIBED = 253
COST_LETHAL = 254
COST_UNKNOWN = 255


def cost254_to_cost100(cost_254: int) -> int:
    """Map canonical [0..254] cost (255=unknown) to OccupancyGrid [-1..100]."""
    if cost_254 == COST_UNKNOWN or cost_254 < 0:
        return -1
    if cost_254 >= COST_LETHAL:
        return 100
    if cost_254 == COST_INSCRIBED:
        return 99
    if cost_254 <= 0:
        return 0
    return max(1, min(98, int(round(cost_254 * 98.0 / 252.0))))


def cost100_to_cost254(cost_100: int) -> int:
    """Invert OccupancyGrid [-1..100] cost back to canonical [0..254] (255=unknown)."""
    if cost_100 < 0:
        return COST_UNKNOWN
    if cost_100 >= 100:
        return COST_LETHAL
    if cost_100 == 99:
        return COST_INSCRIBED
    if cost_100 == 0:
        return COST_FREE
    return max(1, min(252, int(round(cost_100 * 252.0 / 98.0))))


def level_from_z(z_m: float) -> str:
    """Classify robot floor level from world z elevation (meters)."""
    if z_m < 0.28:
        return 'ground'
    if z_m >= 0.78:
        return 'level_2'
    return 'level_1'


def z_from_level(level: str) -> float:
    """Return nominal floor elevation z (meters) for a level name."""
    if level == 'ground':
        return 0.00
    if level == 'level_2':
        return 0.90
    return 0.60


def is_point_restricted_on_l1(
    wx: float,
    wy: float,
    team_side: str = 'red',
    block_l2_footprint: bool = True,
) -> bool:
    """
    Evaluate ABU Robocon 2027 Rulebook & Appendix V1.1 Level 1 structural boundaries
    and restricted zones at world coordinate (wx, wy) in meters.
    """
    side = team_side.strip().lower()

    in_l1_inner = (-2.97 <= wx <= 2.97) and (-2.97 <= wy <= 2.97)
    in_transfer_opening_y = (-1.80 <= wy <= -0.80)
    if side == 'red':
        in_team_transfer = (-4.00 <= wx <= -2.97) and in_transfer_opening_y
    elif side == 'blue':
        in_team_transfer = (2.97 <= wx <= 4.00) and in_transfer_opening_y
    else:
        in_team_transfer = (
            ((-4.00 <= wx <= -2.97) or (2.97 <= wx <= 4.00)) and in_transfer_opening_y
        )

    if not (in_l1_inner or in_team_transfer):
        return True

    if block_l2_footprint and (-1.50 <= wx <= 1.50) and (-1.50 <= wy <= 1.50):
        return True

    if abs(wx) <= 0.015:
        if (-2.22 <= wy <= -1.50) or (1.50 <= wy <= 2.22):
            return True

    in_l1_shared_north = (-0.50 <= wx <= 0.50) and (2.22 <= wy <= 2.97)
    in_l1_shared_south = (-0.50 <= wx <= 0.50) and (-2.97 <= wy <= -2.22)
    in_l1_shared = in_l1_shared_north or in_l1_shared_south

    if side == 'red' and wx > 0.0 and not in_l1_shared:
        return True
    if side == 'blue' and wx < 0.0 and not in_l1_shared:
        return True

    return False


def is_point_restricted_on_level(
    wx: float,
    wy: float,
    level: str = 'level_1',
    team_side: str = 'red',
) -> bool:
    """
    Evaluate ABU Robocon 2027 Rulebook & Appendix V1.1 structural boundaries and
    restricted zones on any of the three competition elevations:
      * 'ground'  (z = 0.00 m): 11x11 m Ground Area, excluding L1 platform walls,
                                Ground Center Dividers, Mustika Pillar, and opponent side
      * 'level_1' (z = 0.60 m): 5940x5940 mm L1 inner floor + team Transfer Area + L1 Shared Areas
      * 'level_2' (z = 0.90 m): 3000x3000 mm L2 Shared Area, excluding outer cliff & Central Pillar
    """
    lvl = level.strip().lower()
    if lvl == 'level_1':
        return is_point_restricted_on_l1(wx, wy, team_side=team_side, block_l2_footprint=True)

    if lvl == 'level_2':
        # Entire 3000 x 3000 mm Level 2 platform is shared by both teams
        if not (-1.50 <= wx <= 1.50 and -1.50 <= wy <= 1.50):
            return True
        # Central Pillar (diameter 270 mm, radius 0.135 m at (0, 0))
        if math.hypot(wx, wy) <= 0.145:
            return True
        return False

    # Ground level (11000 x 11000 mm inner boundary)
    side = team_side.strip().lower()
    if not (-5.50 <= wx <= 5.50 and -5.50 <= wy <= 5.50):
        return True
    # Solid Level 1 base + Stairs/Transfer Area footprint
    if -3.00 <= wx <= 3.00 and -3.00 <= wy <= 3.00:
        return True
    if 3.00 <= abs(wx) <= 4.00 and -2.70 <= wy <= 1.80:
        return True
    # Ground Center Divider fences (30 mm thick)
    if abs(wx) <= 0.015 and (
        (-5.50 <= wy <= -4.85)
        or (-3.65 <= wy <= -3.00)
        or (3.00 <= wy <= 3.75)
        or (4.75 <= wy <= 5.50)
    ):
        return True
    # Mustika Pillar (diameter 270 mm at (0.0, 4.25))
    if math.hypot(wx, wy - 4.25) <= 0.145:
        return True
    # Opponent exclusive Ground Area outside Ground Shared Areas
    in_ground_shared_south = (-0.60 <= wx <= 0.60) and (-4.85 <= wy <= -3.65)
    in_ground_shared_north = (-0.50 <= wx <= 0.50) and (3.75 <= wy <= 4.75)
    if not (in_ground_shared_south or in_ground_shared_north):
        if side == 'red' and wx > 0.0:
            return True
        if side == 'blue' and wx < 0.0:
            return True
    return False


class TraversabilityCostmapGenerator:
    """
    High-speed NumPy/SciPy EDT footprint-inflated traversability costmap generator
    (0..254 canonical scale) supporting Ground, Level 1, and Level 2.
    """

    def __init__(
        self,
        grid_resolution: float = 0.05,
        inscribed_radius: float = 0.35,
        circumscribed_radius: float = 0.50,
        inflation_radius: float = 0.80,
        cost_scaling_factor: float = 5.0,
        team_side: str = 'red',
        enforce_l1_arena_bounds: bool = True,
        active_level: str = 'level_1',
    ) -> None:
        if grid_resolution <= 0.0:
            raise ValueError('grid_resolution must be positive')
        if inscribed_radius < 0.0:
            raise ValueError('inscribed_radius must be non-negative')
        if inflation_radius < inscribed_radius:
            raise ValueError('inflation_radius must be >= inscribed_radius')
        if cost_scaling_factor <= 0.0:
            raise ValueError('cost_scaling_factor must be positive')

        self.resolution = float(grid_resolution)
        self.inscribed_radius = float(inscribed_radius)
        self.circumscribed_radius = float(circumscribed_radius)
        self.inflation_radius = float(inflation_radius)
        self.cost_scaling_factor = float(cost_scaling_factor)
        self.team_side = str(team_side)
        self.enforce_l1_arena_bounds = bool(enforce_l1_arena_bounds)
        self.active_level = str(active_level)

        # Cache of static boolean arena restriction masks keyed by (level, team, w, h, ox, oy, res)
        self._static_mask_cache: Dict[Tuple, np.ndarray] = {}

    def compute_cell_cost_254(self, distance_m: float) -> int:
        """Compute canonical [0..254] traversability cost at Euclidean distance `distance_m`."""
        if distance_m <= 1e-9:
            return COST_LETHAL
        if distance_m <= self.inscribed_radius + 1e-9:
            return COST_INSCRIBED
        if distance_m > self.inflation_radius + 1e-9:
            return COST_FREE

        decay = math.exp(
            -self.cost_scaling_factor * (distance_m - self.inscribed_radius)
        )
        cost = int(math.floor(252.0 * decay))
        return max(1, min(252, cost))

    def _get_static_restricted_mask(
        self, width: int, height: int, origin_x: float, origin_y: float
    ) -> np.ndarray:
        """Compute (and cache) the 2D boolean mask of static restricted cells for the active level."""
        cache_key = (
            self.active_level,
            self.team_side,
            width,
            height,
            round(origin_x, 4),
            round(origin_y, 4),
            round(self.resolution, 5),
        )
        cached = self._static_mask_cache.get(cache_key)
        if cached is not None:
            return cached

        xs = origin_x + (np.arange(width, dtype=np.float64) + 0.5) * self.resolution
        ys = origin_y + (np.arange(height, dtype=np.float64) + 0.5) * self.resolution
        wx, wy = np.meshgrid(xs, ys)  # shape (height, width)

        lvl = self.active_level.strip().lower()
        side = self.team_side.strip().lower()

        if lvl == 'level_1':
            in_l1_inner = (wx >= -2.97) & (wx <= 2.97) & (wy >= -2.97) & (wy <= 2.97)
            in_trans_y = (wy >= -1.80) & (wy <= -0.80)
            if side == 'red':
                in_trans = (wx >= -4.00) & (wx <= -2.97) & in_trans_y
            elif side == 'blue':
                in_trans = (wx >= 2.97) & (wx <= 4.00) & in_trans_y
            else:
                in_trans = (((wx >= -4.00) & (wx <= -2.97)) | ((wx >= 2.97) & (wx <= 4.00))) & in_trans_y

            restricted = ~(in_l1_inner | in_trans)
            restricted |= (wx >= -1.50) & (wx <= 1.50) & (wy >= -1.50) & (wy <= 1.50)
            restricted |= (np.abs(wx) <= 0.015) & (
                ((wy >= -2.22) & (wy <= -1.50)) | ((wy >= 1.50) & (wy <= 2.22))
            )
            in_shared = (np.abs(wx) <= 0.50) & (
                ((wy >= 2.22) & (wy <= 2.97)) | ((wy >= -2.97) & (wy <= -2.22))
            )
            if side == 'red':
                restricted |= (wx > 0.0) & (~in_shared)
            elif side == 'blue':
                restricted |= (wx < 0.0) & (~in_shared)
        elif lvl == 'level_2':
            restricted = ~((wx >= -1.50) & (wx <= 1.50) & (wy >= -1.50) & (wy <= 1.50))
            restricted |= (wx * wx + wy * wy) <= (0.145 * 0.145)
        else:
            # Ground level
            restricted = ~((wx >= -5.50) & (wx <= 5.50) & (wy >= -5.50) & (wy <= 5.50))
            restricted |= (wx >= -3.00) & (wx <= 3.00) & (wy >= -3.00) & (wy <= 3.00)
            restricted |= (np.abs(wx) >= 3.00) & (np.abs(wx) <= 4.00) & (wy >= -2.70) & (wy <= 1.80)
            restricted |= (np.abs(wx) <= 0.015) & (
                ((wy >= -5.50) & (wy <= -4.85))
                | ((wy >= -3.65) & (wy <= -3.00))
                | ((wy >= 3.00) & (wy <= 3.75))
                | ((wy >= 4.75) & (wy <= 5.50))
            )
            restricted |= (wx * wx + (wy - 4.25) ** 2) <= (0.145 * 0.145)
            in_g_shared = (
                ((wx >= -0.60) & (wx <= 0.60) & (wy >= -4.85) & (wy <= -3.65))
                | ((wx >= -0.50) & (wx <= 0.50) & (wy >= 3.75) & (wy <= 4.75))
            )
            if side == 'red':
                restricted |= (wx > 0.0) & (~in_g_shared)
            elif side == 'blue':
                restricted |= (wx < 0.0) & (~in_g_shared)

        self._static_mask_cache[cache_key] = restricted
        return restricted

    def generate_costmap_254(
        self,
        occupancy_data: List[int],
        width: int,
        height: int,
        origin_x: float = -5.7,
        origin_y: float = -5.7,
        occupied_threshold: int = 50,
        preserve_unknown: bool = False,
    ) -> List[int]:
        """
        Vectorized exact EDT computation of the 0..254 traversability costmap
        (~2-4 ms for a 228x228 full-arena grid).
        """
        n_cells = width * height
        if len(occupancy_data) != n_cells:
            raise ValueError(
                f'occupancy_data length ({len(occupancy_data)}) != width*height ({n_cells})'
            )

        occ_arr = np.asarray(occupancy_data, dtype=np.int16).reshape((height, width))
        lethal_mask = occ_arr >= occupied_threshold

        if self.enforce_l1_arena_bounds:
            lethal_mask = lethal_mask | self._get_static_restricted_mask(
                width, height, origin_x, origin_y
            )

        cost_arr = np.zeros((height, width), dtype=np.int16)

        if np.any(lethal_mask):
            # Exact Euclidean distance (meters) from every cell to nearest lethal cell
            dist_m = distance_transform_edt(~lethal_mask) * self.resolution

            # Exponential decay halo (r_inscribed < d <= r_inflation)
            decay_mask = (dist_m > self.inscribed_radius + 1e-9) & (
                dist_m <= self.inflation_radius + 1e-9
            )
            if np.any(decay_mask):
                decay_vals = np.floor(
                    252.0
                    * np.exp(
                        -self.cost_scaling_factor
                        * (dist_m[decay_mask] - self.inscribed_radius)
                    )
                ).astype(np.int16)
                cost_arr[decay_mask] = np.clip(decay_vals, 1, 252)

            # Inscribed footprint zone (0 < d <= r_inscribed)
            inscribed_mask = (~lethal_mask) & (dist_m <= self.inscribed_radius + 1e-9)
            cost_arr[inscribed_mask] = COST_INSCRIBED

            # Lethal cells (d = 0)
            cost_arr[lethal_mask] = COST_LETHAL

        if preserve_unknown:
            unk_mask = (occ_arr < 0) & (cost_arr < COST_INSCRIBED)
            cost_arr[unk_mask] = COST_UNKNOWN

        return cost_arr.ravel().tolist()

    def generate_occupancy_grid_100(
        self,
        occupancy_data: List[int],
        width: int,
        height: int,
        origin_x: float = -5.7,
        origin_y: float = -5.7,
        occupied_threshold: int = 50,
        preserve_unknown: bool = False,
    ) -> Tuple[List[int], List[int]]:
        """
        Return (costmap_254, grid_100) using fast vectorized mapping from [0..254] to [-1..100].
        """
        costmap_254 = self.generate_costmap_254(
            occupancy_data=occupancy_data,
            width=width,
            height=height,
            origin_x=origin_x,
            origin_y=origin_y,
            occupied_threshold=occupied_threshold,
            preserve_unknown=preserve_unknown,
        )
        c_arr = np.asarray(costmap_254, dtype=np.int16)
        g_arr = np.zeros_like(c_arr, dtype=np.int8)

        decay = (c_arr > 0) & (c_arr <= 252)
        if np.any(decay):
            scaled = np.rint(c_arr[decay].astype(np.float64) * (98.0 / 252.0)).astype(np.int16)
            g_arr[decay] = np.clip(scaled, 1, 98).astype(np.int8)

        g_arr[c_arr == COST_INSCRIBED] = 99
        g_arr[c_arr == COST_LETHAL] = 100
        g_arr[(c_arr == COST_UNKNOWN) | (c_arr < 0)] = -1
        return costmap_254, g_arr.tolist()


def build_traversability_viz_buckets(
    costmap_254: List[int],
    width: int,
    height: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> Dict[str, List[Tuple[float, float]]]:
    """
    Partition active-floor traversability zones into clean, non-obscuring visual buckets
    (skipping free space `0` and solid out-of-bounds `254` clutter):
      * 'lethal_edge': 1-cell-thick lethal rim bordering traversable cells (crimson)
      * 'inscribed'  : cost == 253, 0 < d <= r_inscribed (bright cyan collision ring)
      * 'decay_high' : 160 <= cost <= 252 (coral red-orange)
      * 'decay_med'  :  70 <= cost <= 159 (amber orange)
      * 'decay_low'  :   1 <= cost <=  69 (warm yellow outer halo)
    """
    c_arr = np.asarray(costmap_254, dtype=np.int16).reshape((height, width))
    lethal = c_arr == COST_LETHAL
    non_lethal = (c_arr >= 0) & (c_arr < COST_LETHAL)

    # 4-connected border of lethal cells directly touching traversable space
    adj_non_lethal = np.zeros_like(lethal)
    adj_non_lethal[1:, :] |= non_lethal[:-1, :]
    adj_non_lethal[:-1, :] |= non_lethal[1:, :]
    adj_non_lethal[:, 1:] |= non_lethal[:, :-1]
    adj_non_lethal[:, :-1] |= non_lethal[:, 1:]
    lethal_edge = lethal & adj_non_lethal

    masks = {
        'lethal_edge': lethal_edge,
        'inscribed': c_arr == COST_INSCRIBED,
        'decay_high': (c_arr >= 160) & (c_arr <= 252),
        'decay_med': (c_arr >= 70) & (c_arr < 160),
        'decay_low': (c_arr >= 1) & (c_arr < 70),
    }

    buckets: Dict[str, List[Tuple[float, float]]] = {}
    for name, mask in masks.items():
        ys, xs = np.nonzero(mask)
        if xs.size == 0:
            buckets[name] = []
        else:
            wx = origin_x + (xs.astype(np.float64) + 0.5) * resolution
            wy = origin_y + (ys.astype(np.float64) + 0.5) * resolution
            buckets[name] = list(zip(wx.tolist(), wy.tolist()))
    return buckets


class TraversabilityCostmapNode(Node):
    """
    ROS 2 Jazzy node subscribing to /br_occupancy_grid and /odom (for multi-level
    Ground / Level 1 / Level 2 switching) and publishing:
      * /br_traversability_costmap (nav_msgs/OccupancyGrid)
      * /br_traversability_viz     (visualization_msgs/MarkerArray - clean non-pink overlay)
      * /br_footprint_polygon      (geometry_msgs/PolygonStamped)
    """

    def __init__(self) -> None:
        super().__init__('traversability_costmap')

        self.declare_parameter('inflation_radius', 0.80)
        self.declare_parameter('cost_scaling_factor', 5.0)
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('inscribed_radius', 0.35)
        self.declare_parameter('circumscribed_radius', 0.50)
        self.declare_parameter('robot_length_m', 0.7025)
        self.declare_parameter('robot_width_m', 0.6965)
        self.declare_parameter('team_side', 'red')
        self.declare_parameter('enforce_l1_arena_bounds', True)
        self.declare_parameter('base_frame', 'base_link')

        self._latest_occ_grid: Optional[OccupancyGrid] = None
        self._active_level: str = 'ground'
        self._rebuild_generator()
        self.add_on_set_parameters_callback(self._on_parameters_set)

        costmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, '/br_traversability_costmap', costmap_qos
        )
        self.viz_pub = self.create_publisher(
            MarkerArray, '/br_traversability_viz', costmap_qos
        )
        self.footprint_pub = self.create_publisher(
            PolygonStamped, '/br_footprint_polygon', 10
        )
        self.occ_sub = self.create_subscription(
            OccupancyGrid, '/br_occupancy_grid', self._occ_cb, costmap_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10
        )

        self.footprint_timer = self.create_timer(0.1, self._publish_footprint)
        self.republish_timer = self.create_timer(1.0, self._republish_latest_costmap)

        self.get_logger().info(
            f'TraversabilityCostmapNode (Vectorized EDT Multi-Level) initialized: '
            f'inscribed={self.generator.inscribed_radius:.2f}m, '
            f'inflation={self.generator.inflation_radius:.2f}m, '
            f'k={self.generator.cost_scaling_factor:.2f}, team={self.generator.team_side}.'
        )

    def _rebuild_generator(self, overrides: Optional[Dict[str, object]] = None) -> None:
        ov = overrides or {}
        res = float(ov.get('grid_resolution', self.get_parameter('grid_resolution').value))
        if self._latest_occ_grid is not None and 'grid_resolution' not in ov:
            res = float(self._latest_occ_grid.info.resolution)

        inscribed = float(ov.get('inscribed_radius', self.get_parameter('inscribed_radius').value))
        circumscribed = float(
            ov.get('circumscribed_radius', self.get_parameter('circumscribed_radius').value)
        )
        inflation = max(
            inscribed,
            float(ov.get('inflation_radius', self.get_parameter('inflation_radius').value)),
        )
        scaling = float(
            ov.get('cost_scaling_factor', self.get_parameter('cost_scaling_factor').value)
        )
        team_side = str(ov.get('team_side', self.get_parameter('team_side').value))
        enforce_bounds = bool(
            ov.get(
                'enforce_l1_arena_bounds',
                self.get_parameter('enforce_l1_arena_bounds').value,
            )
        )

        self.generator = TraversabilityCostmapGenerator(
            grid_resolution=res,
            inscribed_radius=inscribed,
            circumscribed_radius=circumscribed,
            inflation_radius=inflation,
            cost_scaling_factor=scaling,
            team_side=team_side,
            enforce_l1_arena_bounds=enforce_bounds,
            active_level=self._active_level,
        )

    def _on_parameters_set(self, params) -> SetParametersResult:
        overrides: Dict[str, object] = {}
        for p in params:
            if p.name in ('grid_resolution', 'cost_scaling_factor') and float(p.value) <= 0.0:
                return SetParametersResult(successful=False, reason=f'{p.name} must be > 0')
            if p.name in ('inscribed_radius', 'inflation_radius') and float(p.value) < 0.0:
                return SetParametersResult(successful=False, reason=f'{p.name} must be >= 0')
            overrides[p.name] = p.value

        try:
            self._rebuild_generator(overrides=overrides)
            if self._latest_occ_grid is not None:
                self._compute_and_publish(self._latest_occ_grid)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        return SetParametersResult(successful=True)

    def _odom_cb(self, msg: Odometry) -> None:
        new_level = level_from_z(float(msg.pose.pose.position.z))
        if new_level != self._active_level:
            self._active_level = new_level
            self.generator.active_level = new_level
            self.get_logger().info(
                f'TraversabilityCostmap switched active floor -> {new_level.upper()} (z={z_from_level(new_level):.2f}m)'
            )
            if self._latest_occ_grid is not None:
                self._compute_and_publish(self._latest_occ_grid)

    def _occ_cb(self, msg: OccupancyGrid) -> None:
        self._latest_occ_grid = msg
        # Sync level from occupancy grid origin.z if odom hasn't diverged
        occ_lvl = level_from_z(float(msg.info.origin.position.z))
        if occ_lvl != self._active_level:
            self._active_level = occ_lvl
            self.generator.active_level = occ_lvl
        if abs(self.generator.resolution - float(msg.info.resolution)) > 1e-6:
            self._rebuild_generator()
        self._compute_and_publish(msg)

    def _republish_latest_costmap(self) -> None:
        if self._latest_occ_grid is not None:
            self._compute_and_publish(self._latest_occ_grid)

    def _compute_and_publish(self, occ_msg: OccupancyGrid) -> None:
        width = int(occ_msg.info.width)
        height = int(occ_msg.info.height)
        origin_x = float(occ_msg.info.origin.position.x)
        origin_y = float(occ_msg.info.origin.position.y)
        res = float(occ_msg.info.resolution)

        costmap_254, grid_100 = self.generator.generate_occupancy_grid_100(
            occupancy_data=list(occ_msg.data),
            width=width,
            height=height,
            origin_x=origin_x,
            origin_y=origin_y,
        )

        stamp = self.get_clock().now().to_msg()
        floor_z = z_from_level(self._active_level)

        out = OccupancyGrid()
        out.header.stamp = stamp
        out.header.frame_id = occ_msg.header.frame_id
        out.info = occ_msg.info
        out.info.origin.position.z = floor_z + 0.005
        out.data = grid_100
        self.costmap_pub.publish(out)

        self._publish_traversability_viz(
            costmap_254=costmap_254,
            width=width,
            height=height,
            origin_x=origin_x,
            origin_y=origin_y,
            resolution=res,
            frame_id=occ_msg.header.frame_id,
            stamp=stamp,
            floor_z=floor_z,
        )

    def _publish_traversability_viz(
        self,
        costmap_254: List[int],
        width: int,
        height: int,
        origin_x: float,
        origin_y: float,
        resolution: float,
        frame_id: str,
        stamp,
        floor_z: float,
    ) -> None:
        """
        Publish clean, semi-transparent CUBE_LIST markers for active-floor traversability
        safety zones without blanketing out-of-bounds areas in solid pink.
        """
        buckets = build_traversability_viz_buckets(
            costmap_254=costmap_254,
            width=width,
            height=height,
            origin_x=origin_x,
            origin_y=origin_y,
            resolution=resolution,
        )

        # (marker_id, bucket_key, (r, g, b, a), z_offset)
        layer_styles = (
            (0, 'decay_low', (1.00, 0.92, 0.22, 0.24), 0.012),
            (1, 'decay_med', (1.00, 0.60, 0.10, 0.34), 0.014),
            (2, 'decay_high', (1.00, 0.24, 0.14, 0.44), 0.016),
            (3, 'inscribed', (0.00, 0.90, 1.00, 0.56), 0.018),
            (4, 'lethal_edge', (0.95, 0.12, 0.20, 0.72), 0.020),
        )

        m_array = MarkerArray()
        tile_xy = resolution * 0.96
        for mid, key, (cr, cg, cb, ca), z_off in layer_styles:
            pts = buckets[key]
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame_id
            m.ns = 'traversability_halo'
            m.id = mid
            m.type = Marker.CUBE_LIST
            if not pts:
                m.action = Marker.DELETE
                m_array.markers.append(m)
                continue

            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = tile_xy
            m.scale.y = tile_xy
            m.scale.z = 0.006
            m.color.r = cr
            m.color.g = cg
            m.color.b = cb
            m.color.a = ca
            z_val = float(floor_z + z_off)
            m.points = [Point(x=float(wx), y=float(wy), z=z_val) for wx, wy in pts]
            m_array.markers.append(m)

        self.viz_pub.publish(m_array)

    def _publish_footprint(self) -> None:
        half_l = 0.5 * float(self.get_parameter('robot_length_m').value)
        half_w = 0.5 * float(self.get_parameter('robot_width_m').value)

        poly = PolygonStamped()
        poly.header.stamp = self.get_clock().now().to_msg()
        poly.header.frame_id = str(self.get_parameter('base_frame').value)
        for px, py in ((half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w)):
            pt = Point32()
            pt.x = float(px)
            pt.y = float(py)
            pt.z = 0.02
            poly.polygon.points.append(pt)
        self.footprint_pub.publish(poly)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TraversabilityCostmapNode()
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
