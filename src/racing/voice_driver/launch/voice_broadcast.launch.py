from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Image/text voice broadcast (see docs/VOICE_SETUP.md)."""
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='full'),
        DeclareLaunchArgument('target_sign', default_value='9'),
        Node(
            package='voice_driver',
            executable='voice_broadcast_node',
            name='voice_broadcast_node',
            output='log',
            parameters=[{
                'mode': LaunchConfiguration('mode'),
                'target_sign': LaunchConfiguration('target_sign'),
            }],
        ),
    ])
