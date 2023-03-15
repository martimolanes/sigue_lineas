from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    qt_interface_node = Node(
        package='qt_interface',
        executable='interface',
        parameters=[]
    )
    ld.add_action(qt_interface_node)

    world_node = Node(
        package='world',
        executable='world',
        parameters=[]
    )
    ld.add_action(world_node)

    return ld
