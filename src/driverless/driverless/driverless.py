import rclpy
from rclpy.node import Node
from um_msgs.msg import Waypoints, CarPosition, SteeringCommand
import math

class Driverless(Node):
    def __init__(self):
        super().__init__('driverless')
        self.get_logger().info('driverless node started')

        self.loop_timer = self.create_timer(2.0, self.recalculate_direction_steer)

        self.car_position_subscriber = self.create_subscription(CarPosition, '/car_position', self.on_car_position, 10)
        self.sensed_waypoints_subscriber = self.create_subscription(Waypoints, '/sensed_waypoints', self.on_sensed_waypoints, 10)
        self.cmd_publisher = self.create_publisher(SteeringCommand, '/cmd', 10)

        self.car_position = None
        self.following_waypoint = None

    def on_sensed_waypoints(self, msg):
        new_waypoint = None
        for waypoint in msg.waypoints:
            if new_waypoint == None or waypoint.idx > new_waypoint.idx:
                new_waypoint = waypoint

        self.following_waypoint = new_waypoint

        idx = -1
        if self.following_waypoint:
            idx = self.following_waypoint.idx
        self.get_logger().info('Following waypoint: ' + str(idx))

    def on_car_position(self, msg):
        self.car_position = msg

    def recalculate_direction_steer(self):
        if self.following_waypoint == None or self.car_position == None:
            return

        dx = self.following_waypoint.x - self.car_position.x
        dy = self.following_waypoint.y - self.car_position.y

        angle = math.atan2(dy, dx)
        delta_direction = (angle - self.car_position.direction)
        msg = SteeringCommand(delta_direction=delta_direction)
        self.cmd_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = Driverless()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
