import rclpy
from rclpy.node import Node
import threading

from qt_interface.gui import qt_main

class QtInterface(Node):
    def __init__(self):
        super().__init__('qt_interface')
        self.get_logger().info("qt_interface has started")

def ros_main(args=None):
    rclpy.init(args=args)
    node = QtInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    ros_thread = threading.Thread(target=ros_main)
    ros_thread.start()

    # QT has to be started in the main thread.
    qt_main()
    ros_thread.join()

