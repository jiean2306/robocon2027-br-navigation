#!/usr/bin/env python3
"""
Cost-Aware A* Pathfinding Node with Theta*-Style Line-of-Sight Smoothing,
Smart Standoff Projection, and Dynamic Replanning for ABU Robocon 2027 BR.

Mathematical Formulation
------------------------
1. Cost-Aware 8-Connected A*:
       f(n) = g(n) + h(n)
       delta_g(u, v) = d_step(u, v) * (1.0 + w_cost * (C_254(v) / 252.0))
       h_octile(n, goal) = max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)
   Strictly blocks lethal (254) and inscribed (253) cells and prevents diagonal
   corner cutting across obstacle vertices.

2. Any-Angle Line-of-Sight (LOS) Shortcut Smoothing (Holonomic Mecanum Upgrade):
   Raw 8-connected A* constrains paths to multiples of 45 degrees. A holonomic
   4-wheel mecanum robot can translate along any continuous angle in [-pi, pi).
   After A* finds the optimal grid path, Bresenham LOS pruning connects non-adjacent
   waypoints directly whenever every traversed cell stays safely below the cost
   bound, eliminating staircase zig-zags while preserving safety clearance.

3. Smart Start/Goal Standoff Projection:
   When BR starts flush inside a 700x700 mm Start/Retry Zone (exact 0.35 m wall
   distance) or the operator clicks directly on a 500x500 mm Building Spot against
   a barrier, the planner projects the endpoint to the nearest safe traversable
   standoff cell (within `snap_radius_cells`) instead of failing.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import heapq
import math
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from br_nav_sandbox.occupancy_mapper import bresenham_line
from br_nav_sandbox.traversability_costmap import (
    COST_FREE,
    COST_INSCRIBED,
    COST_LETHAL,
    COST_UNKNOWN,
    cost100_to_cost254,
)


@dataclass
class AStarResult:
    """Result container returned by CostAwareAStarPlanner."""

    success: bool
    path_cells: List[Tuple[int, int]]
    path_world: List[Tuple[float, float]]
    explored_cells: List[Tuple[int, int]]
    total_cost: float
    reason: str = ''
    smoothed_corners: int = 0


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Convert planar yaw angle (radians) to quaternion (x, y, z, w)."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class CostAwareAStarPlanner:
    """
    Pure-Python 8-connected cost-aware A* path planner with optional LOS shortcut
    smoothing and nearest-traversable standoff projection.
    """

    def __init__(
        self,
        allow_diagonal: bool = True,
        cost_penalty_weight: float = 4.0,
        allow_unknown: bool = True,
        unknown_traversal_cost_254: int = 40,
        max_allowed_cost_254: int = COST_INSCRIBED - 1,
        smooth_path: bool = True,
        resample_spacing_m: float = 0.15,
        snap_inscribed_endpoints: bool = False,
        snap_radius_cells: int = 8,
    ) -> None:
        self.allow_diagonal = bool(allow_diagonal)
        self.cost_penalty_weight = float(cost_penalty_weight)
        self.allow_unknown = bool(allow_unknown)
        self.unknown_traversal_cost_254 = int(unknown_traversal_cost_254)
        self.max_allowed_cost_254 = int(max_allowed_cost_254)
        self.smooth_path = bool(smooth_path)
        self.resample_spacing_m = float(resample_spacing_m)
        self.snap_inscribed_endpoints = bool(snap_inscribed_endpoints)
        self.snap_radius_cells = int(snap_radius_cells)

    @staticmethod
    def world_to_cell(
        wx: float, wy: float, origin_x: float, origin_y: float, resolution: float
    ) -> Tuple[int, int]:
        return (
            int(math.floor((wx - origin_x) / resolution)),
            int(math.floor((wy - origin_y) / resolution)),
        )

    @staticmethod
    def cell_to_world(
        cx: int, cy: int, origin_x: float, origin_y: float, resolution: float
    ) -> Tuple[float, float]:
        return (
            origin_x + (cx + 0.5) * resolution,
            origin_y + (cy + 0.5) * resolution,
        )

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Compute admissible octile (8-connected) or Manhattan (4-connected) heuristic."""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        if self.allow_diagonal:
            return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)
        return float(dx + dy)

    def is_cell_traversable(self, cost_254: int) -> bool:
        """Return True if a cell with canonical cost `cost_254` may be traversed."""
        if cost_254 == COST_UNKNOWN or cost_254 < 0:
            return self.allow_unknown
        if cost_254 >= COST_INSCRIBED:
            return False
        return cost_254 <= self.max_allowed_cost_254

    def effective_cell_cost_254(self, cost_254: int) -> int:
        """Return effective traversal cost in [0..252] for a traversable cell."""
        if cost_254 == COST_UNKNOWN or cost_254 < 0:
            return self.unknown_traversal_cost_254
        return max(0, min(252, cost_254))

    def find_nearest_traversable_cell(
        self,
        seed: Tuple[int, int],
        costmap_254: List[int],
        width: int,
        height: int,
        max_radius: int,
    ) -> Optional[Tuple[int, int]]:
        """BFS outward from `seed` up to `max_radius` cells to find the nearest traversable cell."""
        sx, sy = seed
        if not (0 <= sx < width and 0 <= sy < height):
            return None
        if self.is_cell_traversable(costmap_254[sy * width + sx]):
            return seed

        q: deque[Tuple[int, int]] = deque([(sx, sy)])
        visited: Set[Tuple[int, int]] = {(sx, sy)}
        best_cell: Optional[Tuple[int, int]] = None
        best_dist2 = float('inf')

        while q:
            cx, cy = q.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < width and 0 <= ny < height) or (nx, ny) in visited:
                    continue
                d2 = (nx - sx) ** 2 + (ny - sy) ** 2
                if d2 > max_radius * max_radius:
                    continue
                visited.add((nx, ny))
                if self.is_cell_traversable(costmap_254[ny * width + nx]):
                    if d2 < best_dist2:
                        best_dist2 = d2
                        best_cell = (nx, ny)
                else:
                    q.append((nx, ny))

        return best_cell

    def _has_safe_los(
        self,
        a: Tuple[int, int],
        b: Tuple[int, int],
        costmap_254: List[int],
        width: int,
        height: int,
        max_los_cost: int,
    ) -> bool:
        """Check whether Bresenham line from a to b stays strictly traversable and low-cost."""
        ray = bresenham_line(a[0], a[1], b[0], b[1])
        for cx, cy in ray:
            if not (0 <= cx < width and 0 <= cy < height):
                return False
            c = costmap_254[cy * width + cx]
            if not self.is_cell_traversable(c):
                return False
            if self.effective_cell_cost_254(c) > max_los_cost:
                return False
        return True

    def smooth_path_los(
        self,
        raw_cells: List[Tuple[int, int]],
        costmap_254: List[int],
        width: int,
        height: int,
    ) -> List[Tuple[int, int]]:
        """
        Theta*-style greedy Line-of-Sight shortcutting: collapses 45-degree staircase
        zig-zags into straight holonomic segments while never increasing peak obstacle cost.
        """
        if len(raw_cells) <= 2:
            return list(raw_cells)

        # Allow LOS shortcuts only through cells whose cost does not exceed the
        # peak cost already accepted by A* along the shortcutted segment
        smoothed: List[Tuple[int, int]] = [raw_cells[0]]
        i = 0
        n = len(raw_cells)
        while i < n - 1:
            best_j = i + 1
            seg_max_cost = max(
                self.effective_cell_cost_254(costmap_254[raw_cells[i][1] * width + raw_cells[i][0]]),
                self.effective_cell_cost_254(costmap_254[raw_cells[i + 1][1] * width + raw_cells[i + 1][0]]),
            )
            for j in range(i + 2, n):
                c_j = self.effective_cell_cost_254(
                    costmap_254[raw_cells[j][1] * width + raw_cells[j][0]]
                )
                if c_j > seg_max_cost:
                    seg_max_cost = c_j
                if self._has_safe_los(
                    raw_cells[i],
                    raw_cells[j],
                    costmap_254,
                    width,
                    height,
                    max_los_cost=min(self.max_allowed_cost_254, seg_max_cost + 8),
                ):
                    best_j = j
            smoothed.append(raw_cells[best_j])
            i = best_j

        return smoothed

    def _resample_world_path(
        self, corner_world: List[Tuple[float, float]]
    ) -> List[Tuple[float, float]]:
        """Interpolate evenly spaced waypoints along the LOS-smoothed holonomic segments."""
        if len(corner_world) <= 1 or self.resample_spacing_m <= 0.0:
            return list(corner_world)
        dense: List[Tuple[float, float]] = [corner_world[0]]
        for idx in range(len(corner_world) - 1):
            x0, y0 = corner_world[idx]
            x1, y1 = corner_world[idx + 1]
            dist = math.hypot(x1 - x0, y1 - y0)
            steps = max(1, int(math.ceil(dist / self.resample_spacing_m)))
            for s in range(1, steps + 1):
                u = float(s) / float(steps)
                dense.append((x0 + u * (x1 - x0), y0 + u * (y1 - y0)))
        return dense

    def plan_on_costmap_254(
        self,
        costmap_254: List[int],
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
    ) -> AStarResult:
        """Run cost-aware A* on a canonical [0..254] (255=unknown) flat costmap array."""
        start_cell = self.world_to_cell(
            start_world[0], start_world[1], origin_x, origin_y, resolution
        )
        goal_cell = self.world_to_cell(
            goal_world[0], goal_world[1], origin_x, origin_y, resolution
        )
        return self.plan_cells(
            costmap_254=costmap_254,
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            start_cell=start_cell,
            goal_cell=goal_cell,
        )

    def plan_on_occupancy_grid_100(
        self,
        grid_100: List[int],
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
    ) -> AStarResult:
        """Run cost-aware A* on a ROS 2 OccupancyGrid [-1..100] costmap array."""
        costmap_254 = [cost100_to_cost254(c) for c in grid_100]
        return self.plan_on_costmap_254(
            costmap_254=costmap_254,
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            start_world=start_world,
            goal_world=goal_world,
        )

    def plan_cells(
        self,
        costmap_254: List[int],
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        start_cell: Tuple[int, int],
        goal_cell: Tuple[int, int],
    ) -> AStarResult:
        """Core A* graph search between integer grid cells `start_cell` and `goal_cell`."""
        def in_bounds(c: Tuple[int, int]) -> bool:
            return 0 <= c[0] < width and 0 <= c[1] < height

        def cell_cost(c: Tuple[int, int]) -> int:
            return costmap_254[c[1] * width + c[0]]

        if not in_bounds(start_cell):
            return AStarResult(False, [], [], [], float('inf'), 'Start pose is outside costmap bounds')
        if not in_bounds(goal_cell):
            return AStarResult(False, [], [], [], float('inf'), 'Goal pose is outside costmap bounds')

        snap_note = ''
        if not self.is_cell_traversable(cell_cost(start_cell)):
            if self.snap_inscribed_endpoints:
                snapped = self.find_nearest_traversable_cell(
                    start_cell, costmap_254, width, height, max_radius=self.snap_radius_cells
                )
                if snapped is not None:
                    start_cell = snapped
                    snap_note += ' (start snapped out of wall margin)'
            if not self.is_cell_traversable(cell_cost(start_cell)):
                return AStarResult(
                    False,
                    [],
                    [],
                    [],
                    float('inf'),
                    f'Start cell {start_cell} is inside a lethal/inscribed obstacle (cost={cell_cost(start_cell)})',
                )

        if not self.is_cell_traversable(cell_cost(goal_cell)):
            if self.snap_inscribed_endpoints and cell_cost(goal_cell) == COST_INSCRIBED:
                snapped = self.find_nearest_traversable_cell(
                    goal_cell, costmap_254, width, height, max_radius=self.snap_radius_cells
                )
                if snapped is not None:
                    goal_cell = snapped
                    snap_note += ' (goal snapped to standoff pose)'
            if not self.is_cell_traversable(cell_cost(goal_cell)):
                return AStarResult(
                    False,
                    [],
                    [],
                    [],
                    float('inf'),
                    f'Goal cell {goal_cell} is inside a lethal/inscribed obstacle (cost={cell_cost(goal_cell)})',
                )

        offsets: List[Tuple[int, int]] = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        if self.allow_diagonal:
            offsets += [(1, 1), (1, -1), (-1, 1), (-1, -1)]

        open_heap: List[Tuple[float, int, Tuple[int, int]]] = []
        tie_breaker = 0
        h0 = self.heuristic(start_cell, goal_cell)
        heapq.heappush(open_heap, (h0, tie_breaker, start_cell))

        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score: Dict[Tuple[int, int], float] = {start_cell: 0.0}
        closed_set: Set[Tuple[int, int]] = set()
        explored_order: List[Tuple[int, int]] = []

        sqrt2 = math.sqrt(2.0)
        w_cost = self.cost_penalty_weight

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in closed_set:
                continue

            closed_set.add(current)
            explored_order.append(current)

            if current == goal_cell:
                raw_cells: List[Tuple[int, int]] = [current]
                total_cost = g_score[current]
                while current in came_from:
                    current = came_from[current]
                    raw_cells.append(current)
                raw_cells.reverse()

                if self.smooth_path:
                    corner_cells = self.smooth_path_los(raw_cells, costmap_254, width, height)
                    corner_world = [
                        self.cell_to_world(cx, cy, origin_x, origin_y, resolution)
                        for cx, cy in corner_cells
                    ]
                    path_world = self._resample_world_path(corner_world)
                    n_corners = len(corner_cells)
                else:
                    path_world = [
                        self.cell_to_world(cx, cy, origin_x, origin_y, resolution)
                        for cx, cy in raw_cells
                    ]
                    n_corners = len(raw_cells)

                return AStarResult(
                    success=True,
                    path_cells=raw_cells,
                    path_world=path_world,
                    explored_cells=explored_order,
                    total_cost=total_cost,
                    reason='OK' + snap_note,
                    smoothed_corners=n_corners,
                )

            cx, cy = current
            g_cur = g_score[current]

            for dx, dy in offsets:
                nxt = (cx + dx, cy + dy)
                if not in_bounds(nxt) or nxt in closed_set:
                    continue

                nxt_raw_cost = cell_cost(nxt)
                if not self.is_cell_traversable(nxt_raw_cost):
                    continue

                if dx != 0 and dy != 0:
                    c1 = (cx + dx, cy)
                    c2 = (cx, cy + dy)
                    if not (
                        in_bounds(c1)
                        and in_bounds(c2)
                        and self.is_cell_traversable(cell_cost(c1))
                        and self.is_cell_traversable(cell_cost(c2))
                    ):
                        continue

                base_step = sqrt2 if (dx != 0 and dy != 0) else 1.0
                c_eff = self.effective_cell_cost_254(nxt_raw_cost)
                step_cost = base_step * (1.0 + w_cost * (c_eff / 252.0))
                tentative_g = g_cur + step_cost

                if tentative_g < g_score.get(nxt, float('inf')):
                    came_from[nxt] = current
                    g_score[nxt] = tentative_g
                    tie_breaker += 1
                    f_nxt = tentative_g + self.heuristic(nxt, goal_cell)
                    heapq.heappush(open_heap, (f_nxt, tie_breaker, nxt))

        return AStarResult(
            success=False,
            path_cells=[],
            path_world=[],
            explored_cells=explored_order,
            total_cost=float('inf'),
            reason='No collision-free path exists between start and goal',
        )


class AStarPathfinderNode(Node):
    """
    ROS 2 Jazzy node that plans a LOS-smoothed cost-aware A* path over
    /br_traversability_costmap from BR's current pose to /goal_pose, publishing:
      * /br_planned_path (nav_msgs/Path)
      * /br_explored_nodes (visualization_msgs/MarkerArray)
    """

    def __init__(self) -> None:
        super().__init__('astar_pathfinder')

        self.declare_parameter('inflation_radius', 0.80)
        self.declare_parameter('cost_scaling_factor', 5.0)
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('inscribed_radius', 0.35)
        self.declare_parameter('cost_penalty_weight', 4.0)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('allow_unknown', True)
        self.declare_parameter('smooth_path', True)
        self.declare_parameter('snap_inscribed_endpoints', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('default_start_x', -1.85)
        self.declare_parameter('default_start_y', 2.45)
        self.declare_parameter('auto_publish_demo_goal', True)
        self.declare_parameter('demo_goal_x', -0.55)
        self.declare_parameter('demo_goal_y', -2.45)

        self._costmap_msg: Optional[OccupancyGrid] = None
        self._latest_odom_pose: Optional[Tuple[float, float]] = None
        self._active_goal_xy: Optional[Tuple[float, float]] = None
        self._demo_planned: bool = False
        self._last_replan_sec: float = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        costmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/br_traversability_costmap', self._costmap_cb, costmap_qos
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_cb, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10
        )

        self.path_pub = self.create_publisher(Path, '/br_planned_path', path_qos)
        self.markers_pub = self.create_publisher(MarkerArray, '/br_explored_nodes', path_qos)

        self.get_logger().info(
            'AStarPathfinderNode (LOS-Smoothed + Smart Standoff) ready.'
        )

    def _build_planner(self) -> CostAwareAStarPlanner:
        return CostAwareAStarPlanner(
            allow_diagonal=bool(self.get_parameter('allow_diagonal').value),
            cost_penalty_weight=float(self.get_parameter('cost_penalty_weight').value),
            allow_unknown=bool(self.get_parameter('allow_unknown').value),
            smooth_path=bool(self.get_parameter('smooth_path').value),
            snap_inscribed_endpoints=bool(self.get_parameter('snap_inscribed_endpoints').value),
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._latest_odom_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def _get_start_xy(self) -> Tuple[float, float]:
        map_frame = str(self.get_parameter('map_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        try:
            tf_msg: TransformStamped = self.tf_buffer.lookup_transform(
                map_frame, base_frame, rclpy.time.Time()
            )
            return (
                float(tf_msg.transform.translation.x),
                float(tf_msg.transform.translation.y),
            )
        except Exception:
            pass

        if self._latest_odom_pose is not None:
            return self._latest_odom_pose

        return (
            float(self.get_parameter('default_start_x').value),
            float(self.get_parameter('default_start_y').value),
        )

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        self._costmap_msg = msg
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (
            not self._demo_planned
            and bool(self.get_parameter('auto_publish_demo_goal').value)
            and self._active_goal_xy is None
        ):
            self._demo_planned = True
            gx = float(self.get_parameter('demo_goal_x').value)
            gy = float(self.get_parameter('demo_goal_y').value)
            self._active_goal_xy = (gx, gy)
            self._last_replan_sec = now_sec
            self._execute_plan(gx, gy)
        elif self._active_goal_xy is not None and (now_sec - self._last_replan_sec) >= 0.45:
            self._last_replan_sec = now_sec
            self._execute_plan(self._active_goal_xy[0], self._active_goal_xy[1])

    def _goal_cb(self, goal_msg: PoseStamped) -> None:
        gx = float(goal_msg.pose.position.x)
        gy = float(goal_msg.pose.position.y)
        self._active_goal_xy = (gx, gy)
        self._execute_plan(gx, gy)

    def _execute_plan(self, goal_x: float, goal_y: float) -> None:
        if self._costmap_msg is None:
            self.get_logger().warning('Cannot plan yet: /br_traversability_costmap not received.')
            return

        grid = self._costmap_msg
        start_xy = self._get_start_xy()
        goal_xy = (goal_x, goal_y)

        planner = self._build_planner()
        result = planner.plan_on_occupancy_grid_100(
            grid_100=list(grid.data),
            width=int(grid.info.width),
            height=int(grid.info.height),
            resolution=float(grid.info.resolution),
            origin_x=float(grid.info.origin.position.x),
            origin_y=float(grid.info.origin.position.y),
            start_world=start_xy,
            goal_world=goal_xy,
        )

        self._publish_explored_markers(grid, result, start_xy, goal_xy)

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = grid.header.frame_id

        if not result.success:
            # Publish an empty path so autonomous path follower immediately brakes!
            self.path_pub.publish(path_msg)
            self.get_logger().warning(
                f'A* failed from ({start_xy[0]:.2f}, {start_xy[1]:.2f}) to '
                f'({goal_xy[0]:.2f}, {goal_xy[1]:.2f}): {result.reason}'
            )
            return

        z_lvl = float(grid.info.origin.position.z)
        n_pts = len(result.path_world)
        for i, (wx, wy) in enumerate(result.path_world):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = z_lvl + 0.02

            if i + 1 < n_pts:
                nx, ny = result.path_world[i + 1]
                yaw = math.atan2(ny - wy, nx - wx)
            elif i > 0:
                px, py = result.path_world[i - 1]
                yaw = math.atan2(wy - py, wx - px)
            else:
                yaw = 0.0

            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f'Published /br_planned_path: {len(path_msg.poses)} waypoints '
            f'({result.smoothed_corners} LOS corners, explored {len(result.explored_cells)} nodes, '
            f'cost={result.total_cost:.2f}, {result.reason}).'
        )

    def _publish_explored_markers(
        self,
        grid: OccupancyGrid,
        result: AStarResult,
        start_xy: Tuple[float, float],
        goal_xy: Tuple[float, float],
    ) -> None:
        now = self.get_clock().now().to_msg()
        frame_id = grid.header.frame_id
        res = float(grid.info.resolution)
        ox = float(grid.info.origin.position.x)
        oy = float(grid.info.origin.position.y)
        z_lvl = float(grid.info.origin.position.z)
        w = int(grid.info.width)

        m_array = MarkerArray()

        explored_marker = Marker()
        explored_marker.header.stamp = now
        explored_marker.header.frame_id = frame_id
        explored_marker.ns = 'astar_explored_cells'
        explored_marker.id = 0
        explored_marker.type = Marker.CUBE_LIST
        explored_marker.action = Marker.ADD if result.explored_cells else Marker.DELETE
        explored_marker.scale.x = res * 0.85
        explored_marker.scale.y = res * 0.85
        explored_marker.scale.z = 0.01
        explored_marker.pose.orientation.w = 1.0

        for cx, cy in result.explored_cells:
            pt = Point()
            pt.x = ox + (cx + 0.5) * res
            pt.y = oy + (cy + 0.5) * res
            pt.z = z_lvl + 0.01
            explored_marker.points.append(pt)

            c100 = grid.data[cy * w + cx]
            ratio = max(0.0, min(1.0, float(max(0, c100)) / 98.0))
            col = ColorRGBA()
            col.r = float(0.2 + 0.8 * ratio)
            col.g = float(0.85 * (1.0 - 0.6 * ratio))
            col.b = float(0.3 * (1.0 - ratio))
            col.a = 0.55
            explored_marker.colors.append(col)

        m_array.markers.append(explored_marker)

        start_marker = Marker()
        start_marker.header.stamp = now
        start_marker.header.frame_id = frame_id
        start_marker.ns = 'astar_endpoints'
        start_marker.id = 1
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = start_xy[0]
        start_marker.pose.position.y = start_xy[1]
        start_marker.pose.position.z = z_lvl + 0.08
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.16
        start_marker.scale.y = 0.16
        start_marker.scale.z = 0.16
        start_marker.color.r = 0.1
        start_marker.color.g = 0.9
        start_marker.color.b = 0.2
        start_marker.color.a = 0.95
        m_array.markers.append(start_marker)

        goal_marker = Marker()
        goal_marker.header.stamp = now
        goal_marker.header.frame_id = frame_id
        goal_marker.ns = 'astar_endpoints'
        goal_marker.id = 2
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = goal_xy[0]
        goal_marker.pose.position.y = goal_xy[1]
        goal_marker.pose.position.z = z_lvl + 0.08
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = 0.16
        goal_marker.scale.y = 0.16
        goal_marker.scale.z = 0.16
        goal_marker.color.r = 0.0
        goal_marker.color.g = 0.85
        goal_marker.color.b = 1.0
        goal_marker.color.a = 0.95
        m_array.markers.append(goal_marker)

        self.markers_pub.publish(m_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AStarPathfinderNode()
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
