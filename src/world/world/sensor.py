import rclpy
from rclpy.node import Node

from um_msgs.msg import Waypoint

class CarNode(Node):
    def __init__(self):
        super().__init__('car_node')
        self.get_logger().info('car_node node started')

        self.waypoint_publisher = self.create_publisher(Waypoint, '/sensor_waypoints', 10)
        self.waypoint_publisher_timer = self.create_timer(10.0, self.log_things)

    def log_things(self):
        print('hello')

def main(args=None):
    rclpy.init(args=args)

    node = CarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()