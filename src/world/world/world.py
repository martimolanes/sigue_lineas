import rclpy
from rclpy.node import Node

from um_msgs.msg import Waypoints, Waypoint, CarPosition
from world.car import Car

class World(Node):
    def __init__(self):
        super().__init__('world')
        self.get_logger().info('car_node node started')

        self.car = Car()
        self.points = []
        self.generate_waypoints()

        self.waypoint_publisher = self.create_publisher(Waypoints, '/world_waypoints', 10)
        self.waypoint_publisher_timer = self.create_timer(2.0, self.publish_waypoints)

        self.loop_timer = self.create_timer(1.0 / 30.0, self.tick)
        self.car_position_publisher = self.create_publisher(CarPosition, '/car_position', 10)

    def generate_waypoints(self):
        for i in range(10):
            self.points.append(Waypoint(x=0.0, y=i * 30.0))

    def publish_waypoints(self):
        if self.waypoint_publisher.get_subscription_count() >= 1:
            msg = Waypoints(waypoints=self.points)
            self.waypoint_publisher.publish(msg)

    def tick(self):
        self.update_physics()
        self.publish_car_position()

    def update_physics(self):
        self.car.update()

    def publish_car_position(self):
        msg = CarPosition(x=self.car.position[0], y=self.car.position[1], direction=self.car.direction)
        self.car_position_publisher.publish(msg)

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