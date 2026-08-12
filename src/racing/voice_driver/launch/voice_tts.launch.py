from launch import LaunchDescription
import logging

import launch.logging as launch_logging
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from racing_common.launch_status import startup_status


def generate_launch_description() -> LaunchDescription:
    """Start TTS-only node: subscribe ai_description and speak API text."""
    launch_logging.launch_config.level = logging.ERROR
    broadcast_node = Node(
        package='voice_driver',
        executable='voice_broadcast_node',
        name='voice_broadcast_node',
        output='log',
        parameters=[{'mode': LaunchConfiguration('mode')}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='tts_only'),
        broadcast_node,
        RegisterEventHandler(OnProcessStart(
            target_action=broadcast_node,
            on_start=[startup_status('语音节点', '/voice_broadcast_node')],
        )),
    ])
