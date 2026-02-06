from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()

    odom_test_node = Node(
        package='odom_test',
        executable='odom_test_node'
    )
    vex_serial_interface = Node(
        package='vex_serial_interface',
        executable='talker'
    )
    
    foxglove_bridge = ExecuteProcess(cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])

    ld.add_action(odom_test_node)
    ld.add_action(vex_serial_interface)
    ld.add_action(foxglove_bridge)

    return ld