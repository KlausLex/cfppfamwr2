from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'octomap_path',
            # default_value="/opt/ros2_ws/src/collision_environment/geometry/mapfile.bt",
            default_value="/opt/ros2_ws/src/collision_environment/geometry/cabinet.bt",
            description='Path to the octomap file'
        ),

        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_publisher',
            output='screen',
            parameters=[{'octomap_path': LaunchConfiguration('octomap_path')}]
        ),

        Node(
            package='collision_environment',
            executable='load_local_octomap',
            name='load_local_octomap',
            output='screen'
        ),

        # Adding the static_transform_publisher command
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.3', '0', '0', '0', '0', '0.785398163', '0.785398163', 'map', 'world'],
            output='log'
        )
    ])
