import rclpy
from rclpy.node import Node

from um_msgs.msg import Waypoints, Waypoint

class World(Node):
    def __init__(self):
        super().__init__('world')
        self.get_logger().info('car_node node started')

        self.points = []
        self.generate_waypoints()

        self.waypoint_publisher = self.create_publisher(Waypoints, '/world_waypoints', 10)
        self.waypoint_publisher_timer = self.create_timer(2.0, self.publish_waypoints)

    def generate_waypoints(self):
        for i in range(10):
            self.points.append(Waypoint(x=200.0, y=i * 30.0))

    def publish_waypoints(self):
        if self.waypoint_publisher.get_subscription_count() >= 1:
            msg = Waypoints(waypoints=self.points)
            self.waypoint_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = World()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()