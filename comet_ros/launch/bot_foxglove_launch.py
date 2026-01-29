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

    pf_node = Node(
        package='particle_filter',
        executable='particle_filter'
    )
    vex_serial_interface = Node(
        package='vex_serial_interface',
        executable='talker'
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
        )
    )
    
    # foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])
    foxglove_bridge = ExecuteProcess(cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])


    # ld.add_action(rp_lidar_perms)
    # ld.add_action(vex_brain_perms)
    ld.add_action(pf_node)
    ld.add_action(lidar_launch)
    ld.add_action(vex_serial_interface)
    ld.add_action(foxglove_bridge)

    return ld