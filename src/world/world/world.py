import rclpy
from rclpy.node import Node
import numpy as np

from um_msgs.msg import Waypoints, Waypoint, CarPosition, SteeringCommand
from world.car import Car

MIN_WAYPOINT_DISTANCE_SQUARED_TO_CAR = 100.0 ** 2

class World(Node):
    def __init__(self):
        super().__init__('world')
        self.get_logger().info('car_node node started')

        self.car = Car()
        self.waypoints = []
        self.generate_waypoints()

        self.waypoint_publisher = self.create_publisher(Waypoints, '/world_waypoints', 10)
        self.waypoint_publisher_timer = self.create_timer(2.0, self.publish_waypoints)

        self.loop_timer = self.create_timer(1.0 / 30.0, self.tick)
        self.car_position_publisher = self.create_publisher(CarPosition, '/car_position', 10)
        self.sensed_waypoints_publisher = self.create_publisher(Waypoints, '/sensed_waypoints', 10)

        self.cmd_direction_subscriber = self.create_subscription(SteeringCommand, '/cmd', self.on_cmd, 10)

    def generate_waypoints(self):
        width = 350
        height = 150

        steps = 30
        angle_step = 2.0 * np.pi / steps

        for i in range(steps):
            x = np.cos(angle_step * i) * width
            y = np.sin(angle_step * i) * height

            self.waypoints.append(Waypoint(x=x, y=y, idx=i))

    def publish_waypoints(self):
        if self.waypoint_publisher.get_subscription_count() >= 1:
            msg = Waypoints(waypoints=self.waypoints)
            self.waypoint_publisher.publish(msg)

    def tick(self):
        self.update_physics()
        self.publish_car_position()
        self.publish_car_sensed_waypoints()

    def update_physics(self):
        self.car.update()

    def publish_car_position(self):
        msg = CarPosition(x=self.car.position[0], y=self.car.position[1], direction=self.car.direction)
        self.car_position_publisher.publish(msg)

    def publish_car_sensed_waypoints(self):
        # Publish waypoints that are near the car. This is done to simulate a
        # some kind of **sensor** in real life.

        sensed_waypoints = []

        for waypoint in self.waypoints:
            distance_sq = \
                (waypoint.x - self.car.position[0]) ** 2 + \
                (waypoint.y - self.car.position[1]) ** 2

            if distance_sq < MIN_WAYPOINT_DISTANCE_SQUARED_TO_CAR:
                sensed_waypoints.append(waypoint)

        if len(sensed_waypoints) == 0:
            return

        msg = Waypoints(waypoints=sensed_waypoints)
        self.sensed_waypoints_publisher.publish(msg)

    def on_cmd(self, msg):
        self.car.direction += msg.delta_direction


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