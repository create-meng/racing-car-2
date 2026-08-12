from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import launch_ros.actions
from racing_common.launch_status import startup_status

def generate_launch_description():
    akmcar = LaunchConfiguration('akmcar', default='false')
    base_config = Path(get_package_share_directory('origincar_base'), 'config', 'base.yaml')

    robot_parameters = [
        {'usart_port_name': '/dev/ttyACM0',
         'serial_baud_rate': 115200,
         'robot_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
         'cmd_vel': 'cmd_vel',
         'cmd_vel_watchdog_enabled': True,
         'cmd_vel_watchdog_timeout_sec': 0.35,
         'product_number': 0}
    ]

    origincar_base_node = launch_ros.actions.Node(
        condition=UnlessCondition(akmcar),
        package='origincar_base',
        executable='origincar_base_node',
        parameters=[base_config] + robot_parameters + [{'akm_cmd_vel': 'none'}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'akmcar',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        launch_ros.actions.Node(
            condition=IfCondition(akmcar),
            package='origincar_base',
            executable='origincar_base_node',
            parameters=[base_config] + robot_parameters + [{'akm_cmd_vel': 'ackermann_cmd'}],
            remappings=[('/cmd_vel', 'cmd_vel')],
        ),

        launch_ros.actions.Node(
            condition=IfCondition(akmcar),
            package='origincar_base',
            executable='cmd_vel_to_ackermann_drive.py',
            name='cmd_vel_to_ackermann_drive',
        ),

        origincar_base_node,
        RegisterEventHandler(OnProcessStart(
            target_action=origincar_base_node,
            on_start=[startup_status('底盘', '/origincar_base')],
        )),
    ])
