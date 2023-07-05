import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='Container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ctrl_bridge',
                plugin='CtrlBridge',
                name='check'
            )
        ],
        output='screen'
    )
    return launch.LaunchDescription([
        container
        ])
