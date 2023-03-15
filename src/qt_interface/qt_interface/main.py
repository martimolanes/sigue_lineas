import rclpy
from rclpy.node import Node

import threading
from queue import Queue

from qt_interface.gui import qt_main
from qt_interface.communication import Message, INITIAL_WAYPOINT_CONFIG, UPDATE_CAR_POSITION
from um_msgs.msg import Waypoints, CarPosition

class QtInterface(Node):
    def __init__(self, qt_queue_out):
        super().__init__('qt_interface')
        self.get_logger().info("qt_interface has started")
        self.initial_configuration_subscriber = self.create_subscription(Waypoints, '/world_waypoints', self.on_world_waypoints_published, 10)
        self.received_waypoints = False
        self.qt_queue_out = qt_queue_out

        self.car_position_subscriber = self.create_subscription(CarPosition, '/car_position', self.on_car_position_published, 10)

    def on_world_waypoints_published(self, msg):
        if self.received_waypoints:
            return
        self.get_logger().info('Received initial waypoints')
        self.received_waypoints = True
        self.qt_queue_out.put(Message(INITIAL_WAYPOINT_CONFIG, msg.waypoints))

    def on_car_position_published(self, msg):
        self.qt_queue_out.put(Message(UPDATE_CAR_POSITION, msg))

def ros_main(queue):
    rclpy.init(args=None)
    node = QtInterface(queue)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    q = Queue()

    ros_thread = threading.Thread(target=ros_main, args=(q, ))
    ros_thread.start()

    # QT has to be started in the main thread.
    qt_main(q)
    ros_thread.join()

