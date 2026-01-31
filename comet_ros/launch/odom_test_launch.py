import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # rp_lidar_perms = ExecuteProcess(cmd=["chmod", "777", "/dev/rplidar"], shell=True, prefix="sudo")
    # vex_brain_perms = ExecuteProcess(cmd=["chmod", "777", "/dev/vexbrain"], shell=True, prefix="sudo")

    odom_test_node = Node(
        package='odom_test',
        executable='odom_test_node'
    )
    vex_serial_interface = Node(
        package='vex_serial_interface',
        executable='talker'
    )
    
    # foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])
    foxglove_bridge = ExecuteProcess(cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])


    # ld.add_action(rp_lidar_perms)
    # ld.add_action(vex_brain_perms)
    ld.add_action(odom_test_node)
    ld.add_action(vex_serial_interface)
    ld.add_action(foxglove_bridge)

    return ld