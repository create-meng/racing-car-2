import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from racing_common.launch_status import startup_status


def generate_launch_description():
    bringup_dir = get_package_share_directory('origincar_bringup')
    default_map_yaml = os.path.join(bringup_dir, 'map', 'map_restricted.yaml')

    map_yaml_arg = DeclareLaunchArgument('map_yaml', default_value=default_map_yaml)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    map_frame_arg = DeclareLaunchArgument('map_frame', default_value='map')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom_combined')
    map_to_odom_x_arg = DeclareLaunchArgument('map_to_odom_x', default_value='0.50')
    map_to_odom_y_arg = DeclareLaunchArgument('map_to_odom_y', default_value='0.20')
    map_to_odom_yaw_arg = DeclareLaunchArgument('map_to_odom_yaw', default_value='0.1745329252')
    publish_map_to_odom_arg = DeclareLaunchArgument(
        'publish_map_to_odom', default_value='true',
        description='Publish fixed map->odom TF; production startup localization disables this'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_yaml'),
            'frame_id': LaunchConfiguration('map_frame'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_overlay',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_combined',
        arguments=[
            '--x',
            LaunchConfiguration('map_to_odom_x'),
            '--y',
            LaunchConfiguration('map_to_odom_y'),
            '--z', '0.0',
            '--yaw',
            LaunchConfiguration('map_to_odom_yaw'),
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id',
            LaunchConfiguration('map_frame'),
            '--child-frame-id',
            LaunchConfiguration('odom_frame'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_map_to_odom')),
    )

    return LaunchDescription([
        map_yaml_arg,
        use_sim_time_arg,
        map_frame_arg,
        odom_frame_arg,
        map_to_odom_x_arg,
        map_to_odom_y_arg,
        map_to_odom_yaw_arg,
        publish_map_to_odom_arg,
        map_server,
        RegisterEventHandler(OnProcessStart(
            target_action=map_server,
            on_start=[startup_status('地图服务', '/map_server')],
        )),
        lifecycle_manager,
        map_to_odom,
        RegisterEventHandler(OnProcessStart(
            target_action=map_to_odom,
            on_start=[startup_status('地图 TF', '/map_to_odom_combined')],
        )),
    ])
