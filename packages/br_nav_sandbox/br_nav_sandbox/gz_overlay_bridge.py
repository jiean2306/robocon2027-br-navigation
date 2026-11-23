#!/usr/bin/env python3
"""
Gazebo Harmonic Live 3D Navigation Overlay Bridge for ABU Robocon 2027 BR.

Renders live ROS 2 navigation layers directly inside the Gazebo Sim 3D viewport
using Gazebo's built-in MarkerManager `/marker_array` service (`gz.msgs.Marker_V`):
  1. Traversability Costmap Halo (`/br_traversability_viz`):
       * Bright Cyan Inscribed Collision Ring (0 < d <= 0.35 m)
       * Red -> Orange -> Yellow Exponential Decay Buffer (0.35 m < d <= 0.80 m)
       * Crisp Crimson Lethal Obstacle Perimeter (walls, Earth/Sky blocks, Match Robots)
  2. Live LiDAR Dynamic Obstacles (`/br_dynamic_obstacles`):
       * Glowing Magenta dynamic detections (live LiDAR diff vs static map)
  3. A* Planned Path (`/br_planned_path`):
       * Neon-Green 3D trajectory ribbon directly on the active arena floor
  4. Match Robots (`/br_enemy_markers`):
       * Autonomous OUR TR (Red), ENEMY BR (Blue), and ENEMY TR (Cyan-Blue)
"""

from __future__ import annotations

import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

try:
    import gz.transport13 as gzt
    from gz.msgs10 import boolean_pb2, marker_pb2, marker_v_pb2

    GZ_AVAILABLE = True
except Exception:
    gzt = None
    boolean_pb2 = None
    marker_pb2 = None
    marker_v_pb2 = None
    GZ_AVAILABLE = False


def _set_gz_material(gz_marker, r: float, g: float, b: float, a: float, emissive_scale: float = 0.65) -> None:
    """Populate ambient, diffuse, and emissive RGBA colors on a gz.msgs.Marker."""
    for ch in (gz_marker.material.ambient, gz_marker.material.diffuse):
        ch.r = float(r)
        ch.g = float(g)
        ch.b = float(b)
        ch.a = float(a)
    gz_marker.material.emissive.r = float(r * emissive_scale)
    gz_marker.material.emissive.g = float(g * emissive_scale)
    gz_marker.material.emissive.b = float(b * emissive_scale)
    gz_marker.material.emissive.a = float(a)


def build_gz_quad_list_marker(
    ns: str,
    marker_id: int,
    points_xyz: List[Tuple[float, float, float]],
    tile_size: float,
    rgba: Tuple[float, float, float, float],
) -> object:
    """
    Build a Gazebo `TRIANGLE_LIST` marker rendering flat horizontal quads at each (x, y, z).
    Returns a `gz.msgs.Marker` (ADD_MODIFY if points exist, DELETE_MARKER if empty).
    """
    m = marker_pb2.Marker()
    m.ns = ns
    m.id = int(marker_id)
    if not points_xyz:
        m.action = marker_pb2.Marker.DELETE_MARKER
        return m

    m.action = marker_pb2.Marker.ADD_MODIFY
    m.type = marker_pb2.Marker.TRIANGLE_LIST
    _set_gz_material(m, *rgba)

    half = 0.5 * float(tile_size)
    for cx, cy, cz in points_xyz:
        x0, x1 = cx - half, cx + half
        y0, y1 = cy - half, cy + half
        # Two CCW triangles facing +Z
        for vx, vy in ((x0, y0), (x1, y0), (x1, y1), (x0, y0), (x1, y1), (x0, y1)):
            p = m.point.add()
            p.x = float(vx)
            p.y = float(vy)
            p.z = float(cz)
    return m


def build_gz_path_marker(
    ns: str,
    marker_id: int,
    waypoints_xyz: List[Tuple[float, float, float]],
    rgba: Tuple[float, float, float, float] = (0.0, 1.0, 0.45, 1.0),
) -> object:
    """Build a Gazebo `LINE_STRIP` marker for the A* planned trajectory."""
    m = marker_pb2.Marker()
    m.ns = ns
    m.id = int(marker_id)
    if len(waypoints_xyz) < 2:
        m.action = marker_pb2.Marker.DELETE_MARKER
        return m

    m.action = marker_pb2.Marker.ADD_MODIFY
    m.type = marker_pb2.Marker.LINE_STRIP
    _set_gz_material(m, *rgba, emissive_scale=0.9)
    for wx, wy, wz in waypoints_xyz:
        p = m.point.add()
        p.x = float(wx)
        p.y = float(wy)
        p.z = float(wz)
    return m


class GazeboOverlayBridgeNode(Node):
    """
    Non-blocking ROS 2 -> Gazebo Harmonic MarkerManager bridge.
    Automatically activates whenever Gazebo Sim GUI advertises `/marker_array`.
    """

    def __init__(self) -> None:
        super().__init__('gz_overlay_bridge')
        self.declare_parameter('enabled', True)
        self.declare_parameter('update_hz', 4.0)

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._dirty_trav = False
        self._dirty_dyn = False
        self._dirty_path = False
        self._dirty_robots = False
        self._dirty_mission_objs = False

        self._latest_trav: Optional[MarkerArray] = None
        self._latest_dyn: Optional[MarkerArray] = None
        self._latest_path: Optional[Path] = None
        self._latest_robots: Optional[MarkerArray] = None
        self._latest_mission_objs: Optional[MarkerArray] = None

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.trav_sub = self.create_subscription(
            MarkerArray, '/br_traversability_viz', self._trav_cb, latched_qos
        )
        self.dyn_sub = self.create_subscription(
            MarkerArray, '/br_dynamic_obstacles', self._dyn_cb, 10
        )
        self.path_sub = self.create_subscription(
            Path, '/br_planned_path', self._path_cb, latched_qos
        )
        self.robots_sub = self.create_subscription(
            MarkerArray, '/br_enemy_markers', self._robots_cb, 10
        )
        self.mission_objs_sub = self.create_subscription(
            MarkerArray, '/br_mission_objects', self._mission_objs_cb, latched_qos
        )

        self._gz_node = gzt.Node() if (GZ_AVAILABLE and bool(self.get_parameter('enabled').value)) else None
        self._gz_ready = False

        if self._gz_node is not None:
            self._worker = threading.Thread(target=self._gz_publish_loop, daemon=True)
            self._worker.start()
            self.get_logger().info(
                'GazeboOverlayBridgeNode started: waiting for Gazebo Sim `/marker_array` service...'
            )
        else:
            self._worker = None
            self.get_logger().info('GazeboOverlayBridgeNode idle (Gazebo Python transport unavailable or disabled).')

    def _trav_cb(self, msg: MarkerArray) -> None:
        with self._lock:
            self._latest_trav = msg
            self._dirty_trav = True

    def _dyn_cb(self, msg: MarkerArray) -> None:
        with self._lock:
            self._latest_dyn = msg
            self._dirty_dyn = True

    def _path_cb(self, msg: Path) -> None:
        with self._lock:
            self._latest_path = msg
            self._dirty_path = True

    def _robots_cb(self, msg: MarkerArray) -> None:
        with self._lock:
            self._latest_robots = msg
            self._dirty_robots = True

    def _mission_objs_cb(self, msg: MarkerArray) -> None:
        with self._lock:
            self._latest_mission_objs = msg
            self._dirty_mission_objs = True

    def _send_marker_v(self, mv) -> bool:
        if self._gz_node is None or len(mv.marker) == 0:
            return False
        try:
            ok, _rep = self._gz_node.request(
                '/marker_array', mv, marker_v_pb2.Marker_V, boolean_pb2.Boolean, 250
            )
            return bool(ok)
        except Exception:
            return False

    def _gz_publish_loop(self) -> None:
        period = 1.0 / max(0.5, float(self.get_parameter('update_hz').value))
        last_service_probe = 0.0
        last_trav_push = 0.0

        while not self._stop_event.is_set():
            now = time.monotonic()

            if not self._gz_ready:
                if now - last_service_probe >= 1.0:
                    last_service_probe = now
                    try:
                        services = self._gz_node.service_list()
                        if '/marker_array' in services:
                            self._gz_ready = True
                            with self._lock:
                                self._dirty_trav = self._latest_trav is not None
                                self._dirty_dyn = self._latest_dyn is not None
                                self._dirty_path = self._latest_path is not None
                                self._dirty_robots = self._latest_robots is not None
                                self._dirty_mission_objs = self._latest_mission_objs is not None
                            self.get_logger().info(
                                'Connected to Gazebo Sim `/marker_array`! Streaming live Traversability, '
                                'Occupancy Dynamic Obstacles, A* Path, Match Robots & Mission Objects into Gazebo 3D view.'
                            )
                    except Exception:
                        pass
                time.sleep(0.25)
                continue

            trav_msg = dyn_msg = path_msg = robots_msg = mission_msg = None
            with self._lock:
                if self._dirty_trav and (now - last_trav_push >= 0.45):
                    trav_msg = self._latest_trav
                    self._dirty_trav = False
                    last_trav_push = now
                if self._dirty_dyn:
                    dyn_msg = self._latest_dyn
                    self._dirty_dyn = False
                if self._dirty_path:
                    path_msg = self._latest_path
                    self._dirty_path = False
                if self._dirty_robots:
                    robots_msg = self._latest_robots
                    self._dirty_robots = False
                if self._dirty_mission_objs:
                    mission_msg = self._latest_mission_objs
                    self._dirty_mission_objs = False

            mv = marker_v_pb2.Marker_V()

            # 1. Traversability Costmap Halo (inscribed cyan + red/orange/yellow decay + crimson edge)
            if trav_msg is not None:
                for rm in trav_msg.markers:
                    pts = [(p.x, p.y, p.z + 0.008) for p in rm.points]
                    if rm.action == Marker.DELETE or not pts:
                        gz_m = build_gz_quad_list_marker('br_traversability', rm.id, [], 0.05, (0, 0, 0, 0))
                    else:
                        rgba = (rm.color.r, rm.color.g, rm.color.b, min(0.75, rm.color.a + 0.08))
                        tile = max(0.03, float(rm.scale.x))
                        gz_m = build_gz_quad_list_marker('br_traversability', rm.id, pts, tile, rgba)
                    mv.marker.add().CopyFrom(gz_m)

            # 2. Live LiDAR Dynamic Obstacles (diff vs static pre-made map)
            if dyn_msg is not None:
                dyn_pts: List[Tuple[float, float, float]] = []
                for rm in dyn_msg.markers:
                    if rm.action != Marker.DELETE:
                        dyn_pts.extend((p.x, p.y, p.z + 0.06) for p in rm.points)
                gz_dyn = build_gz_quad_list_marker(
                    'br_occupancy_dyn', 10, dyn_pts, 0.06, (1.0, 0.08, 0.85, 0.92)
                )
                mv.marker.add().CopyFrom(gz_dyn)

            # 3. A* Planned Path
            if path_msg is not None:
                wps = [
                    (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z + 0.07)
                    for ps in path_msg.poses
                ]
                gz_path = build_gz_path_marker('br_astar_path', 20, wps, (0.0, 1.0, 0.45, 1.0))
                mv.marker.add().CopyFrom(gz_path)

            # 4. Match Robots (OUR TR, ENEMY BR, ENEMY TR)
            if robots_msg is not None:
                for rm in robots_msg.markers:
                    if rm.type not in (Marker.CUBE, Marker.CYLINDER):
                        continue
                    gz_r = marker_pb2.Marker()
                    gz_r.ns = 'br_match_robots'
                    gz_r.id = 30 + int(rm.id)
                    if rm.action == Marker.DELETE:
                        gz_r.action = marker_pb2.Marker.DELETE_MARKER
                    else:
                        gz_r.action = marker_pb2.Marker.ADD_MODIFY
                        gz_r.type = (
                            marker_pb2.Marker.BOX
                            if rm.type == Marker.CUBE
                            else marker_pb2.Marker.CYLINDER
                        )
                        gz_r.pose.position.x = float(rm.pose.position.x)
                        gz_r.pose.position.y = float(rm.pose.position.y)
                        gz_r.pose.position.z = float(rm.pose.position.z)
                        gz_r.pose.orientation.w = 1.0
                        gz_r.scale.x = float(rm.scale.x)
                        gz_r.scale.y = float(rm.scale.y)
                        gz_r.scale.z = float(rm.scale.z)
                        _set_gz_material(
                            gz_r, rm.color.r, rm.color.g, rm.color.b, rm.color.a, emissive_scale=0.35
                        )
                    mv.marker.add().CopyFrom(gz_r)

            # 5. Live Mission Game Objects (Carried/Stacked Earth Blocks, Sky Blocks, Golden Mustika)
            if mission_msg is not None:
                for rm in mission_msg.markers:
                    if rm.type not in (Marker.CUBE, Marker.SPHERE):
                        continue
                    gz_o = marker_pb2.Marker()
                    gz_o.ns = 'br_mission_objects'
                    gz_o.id = 200 + int(rm.id)
                    if rm.action == Marker.DELETE:
                        gz_o.action = marker_pb2.Marker.DELETE_MARKER
                    else:
                        gz_o.action = marker_pb2.Marker.ADD_MODIFY
                        gz_o.type = (
                            marker_pb2.Marker.BOX
                            if rm.type == Marker.CUBE
                            else marker_pb2.Marker.SPHERE
                        )
                        gz_o.pose.position.x = float(rm.pose.position.x)
                        gz_o.pose.position.y = float(rm.pose.position.y)
                        gz_o.pose.position.z = float(rm.pose.position.z)
                        gz_o.pose.orientation.w = 1.0
                        gz_o.scale.x = float(rm.scale.x)
                        gz_o.scale.y = float(rm.scale.y)
                        gz_o.scale.z = float(rm.scale.z)
                        _set_gz_material(
                            gz_o, rm.color.r, rm.color.g, rm.color.b, rm.color.a, emissive_scale=0.45
                        )
                    mv.marker.add().CopyFrom(gz_o)

            if len(mv.marker) > 0:
                if not self._send_marker_v(mv):
                    # Gazebo window may have closed or restarted; re-probe cleanly
                    self._gz_ready = False

            time.sleep(period)

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=0.5)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboOverlayBridgeNode()
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
