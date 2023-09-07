import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('helios_control_bring_up'), 'descriptions', 'urdf', 'gm6020.urdf.xacro'
    )])

    robot_controller = PathJoinSubstitution(
        [
            FindPackageShare(
            "helios_control_bring_up"
            ),
            "config",
            "gm6020.controllers.yaml"
        ]
    )

    controller_manager=Node(
        package='controller_manager',
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, robot_controller],
        output="screen",
        emulate_tty=True
    )

    controller_spwaner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gm6020_control", "-c", "controller_manager"]
    )   

    nodes = [controller_manager, controller_spwaner]

    return LaunchDescription(nodes)
