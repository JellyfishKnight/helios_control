from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch

def generate_launch_description():
    config_file_root = os.path.join(get_package_share_directory('bring_up'), 'config', 'Infantry3.yaml')
    params_file = os.path.join(get_package_share_directory('mindvision_camera'), 'config', 'camera_params.yaml')

    camera_info_url = 'package://mindvision_camera/config/camera_info.yaml'

    return launch.LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        DeclareLaunchArgument(name='camera_info_url',
                              default_value=camera_info_url),
        DeclareLaunchArgument(name='use_sensor_data_qos',
                              default_value='false'),
        DeclareLaunchArgument(name='config_file_root',
                              default_value=config_file_root),
        Node(
            package='mindvision_camera',
            executable='mindvision_camera_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
            }],
        ),
        Node(
            package='ctrl_bridge',
            executable='ctrl_bridge_node',
        ),
        Node(
            package='armor_predictor',
            executable='armor_predictor_node',
            emulate_tty=True,
            parameters=[config_file_root],
        ),
        Node(
            package='armor_detector',
            executable='armor_detector_node',
            emulate_tty=True,
            # parameters=[config_file_root],
        ),
        Node(
            package='armor_autofire',
            executable='armor_autofire_node',
            output='screen',
        ),
        # Node(
        #     package="energy_detector",
        #     executable="energy_detector",
        #     output="screen"
        # ),
        # Node(
        #     package="energy_pre",
        #     executable="energy_predict",
        #     output="screen"
        # ),
        # Node(
        #     package="energy_pre",
        #     executable="ceres_solve",
        #     output="screen"
        # ),
        # Node(
        #     package="recorder",
        #     executable="recorder_node",
        #     output="screen"
        # )
    ])
