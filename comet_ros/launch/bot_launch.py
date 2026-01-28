import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    pf_node = Node(
        package='particle_filter',
        executable='particle_filter'
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
        )
    )
    vex_serial_interface = Node(
        package='vex_serial_interface',
        executable='talker'
    )

    ld.add_action(pf_node)
    ld.add_action(lidar_launch)
    ld.add_action(vex_serial_interface)

    return ld