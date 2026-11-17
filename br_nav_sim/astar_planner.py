import heapq
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('start_x', 1.0)
        self.declare_parameter('start_y', 1.0)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('occupied_threshold', 50)
        self.map_msg = None
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 1)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.get_logger().info('A* ready. Waiting for /map and /goal_pose.')

    def map_cb(self, msg):
        self.map_msg = msg

    def goal_cb(self, goal):
        if self.map_msg is None:
            self.get_logger().warning('No map received yet.')
            return
        path_cells = self.plan(goal.pose.position.x, goal.pose.position.y)
        if not path_cells:
            self.get_logger().warning('No valid path found to requested goal.')
            return

        grid = self.map_msg
        out = Path()
        out.header.frame_id = grid.header.frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        for cx, cy in path_cells:
            pose = PoseStamped()
            pose.header = out.header
            pose.pose.position.x = grid.info.origin.position.x + (cx + 0.5) * grid.info.resolution
            pose.pose.position.y = grid.info.origin.position.y + (cy + 0.5) * grid.info.resolution
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            out.poses.append(pose)
        self.path_pub.publish(out)
        self.get_logger().info(f'Published A* path with {len(out.poses)} poses.')

    def plan(self, gx_m, gy_m):
        grid = self.map_msg
        w, h = int(grid.info.width), int(grid.info.height)
        res = float(grid.info.resolution)
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y

        def world_to_cell(x, y):
            return (int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res)))

        start = world_to_cell(
            float(self.get_parameter('start_x').value),
            float(self.get_parameter('start_y').value))
        goal = world_to_cell(gx_m, gy_m)
        if not self.free(start, w, h) or not self.free(goal, w, h):
            return []

        diagonal = bool(self.get_parameter('allow_diagonal').value)
        threshold = int(self.get_parameter('occupied_threshold').value)
        offsets = [(1,0),(-1,0),(0,1),(0,-1)]
        if diagonal:
            offsets += [(1,1),(1,-1),(-1,1),(-1,-1)]

        def traversable(cell):
            x, y = cell
            if not self.free(cell, w, h):
                return False
            return grid.data[y * w + x] < threshold

        open_heap = [(0.0, start)]
        came_from = {}
        g = {start: 0.0}
        closed = set()

        def heuristic(a, b):
            dx, dy = abs(a[0]-b[0]), abs(a[1]-b[1])
            if diagonal:
                return max(dx, dy) + (math.sqrt(2.0)-1.0)*min(dx, dy)
            return dx + dy

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal:
                route = [current]
                while current in came_from:
                    current = came_from[current]
                    route.append(current)
                return list(reversed(route))
            closed.add(current)

            for dx, dy in offsets:
                nxt = (current[0]+dx, current[1]+dy)
                if not traversable(nxt) or nxt in closed:
                    continue
                # Prevent cutting diagonally through the corner of an obstacle.
                if dx and dy:
                    if not traversable((current[0]+dx, current[1])) or not traversable((current[0], current[1]+dy)):
                        continue
                step = math.sqrt(2.0) if dx and dy else 1.0
                tentative = g[current] + step
                if tentative < g.get(nxt, float('inf')):
                    came_from[nxt] = current
                    g[nxt] = tentative
                    heapq.heappush(open_heap, (tentative + heuristic(nxt, goal), nxt))
        return []

    @staticmethod
    def free(cell, width, height):
        x, y = cell
        return 0 <= x < width and 0 <= y < height


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
