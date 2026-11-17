import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class FieldMapNode(Node):
    """Publish a baseline 6 m x 6 m L1 occupancy grid.

    Coordinates are local to L1: map origin is the lower-left corner of the
    L1 floor; x points along the field's chosen forward axis, y to the left.
    The map is deliberately not yet populated with all field furniture.
    """

    def __init__(self):
        super().__init__('field_map')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width_m', 6.0)
        self.declare_parameter('height_m', 6.0)
        self.declare_parameter('boundary_cells', 1)

        self.pub = self.create_publisher(OccupancyGrid, '/map', 1)
        self.publish_map()

    def publish_map(self):
        res = float(self.get_parameter('resolution').value)
        width_m = float(self.get_parameter('width_m').value)
        height_m = float(self.get_parameter('height_m').value)
        border = int(self.get_parameter('boundary_cells').value)
        width = int(round(width_m / res))
        height = int(round(height_m / res))

        msg = OccupancyGrid()
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        msg.info.resolution = res
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0

        data = [0] * (width * height)
        for y in range(height):
            for x in range(width):
                if x < border or y < border or x >= width - border or y >= height - border:
                    data[y * width + x] = 100
        msg.data = data
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.get_logger().info(
            f'Published baseline L1 map: {width}x{height} cells at {res:.3f} m/cell'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FieldMapNode()
    # Publish once more after discovery so late subscribers receive the map.
    timer = node.create_timer(1.0, node.publish_map)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
