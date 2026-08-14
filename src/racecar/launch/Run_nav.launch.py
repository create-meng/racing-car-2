import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 组合模式下定位生命周期与 Nav2 节点共享隔离容器，避免独立进程启动时
    # map_server/change_state 在资源竞争下超时。
    use_composition = LaunchConfiguration('use_composition', default='True')

        
    # nav_dir = get_package_share_directory('racecar')
    # nav_launchr = os.path.join(nav_dir, 'launch')

    racecar_dir = get_package_share_directory('racecar')
    racecar_launchr = os.path.join(racecar_dir, 'launch')

    map_dir = os.path.join(racecar_dir, 'map')
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, '/root/ros2_ws/map/ai_map.yaml'))
 
    param_dir = os.path.join(racecar_dir, 'config')
    param_file = LaunchConfiguration('params', default=os.path.join(
        param_dir, 'nav.yaml'))

    # 诊断监控脚本路径（触发"乱飘"时自动写日志）
    trigger_monitor_script = '/root/ros2_ws/src/racecar/scripts/trigger_monitor.py'


    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='Use a shared Nav2 component container'),
        

        Node(
            name='waypoint_cycle',
            package='nav2_waypoint_cycle',
            executable='nav2_waypoint_cycle',
        ),

        # 自动启动诊断监控（写日志，不影响导航）
        ExecuteProcess(
            cmd=['python3', trigger_monitor_script],
            name='trigger_monitor',
            output='screen',
        ),

        Node(
            package='racecar',
            executable='static_wall_scan_filter.py',
            name='static_wall_scan_filter',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'filtered_topic': '/scan_nav',
                'map_topic': '/map',
                'map_frame': 'map',
                'match_tolerance': 0.10,
                'map_cell_radius': 1,
                'occupied_threshold': 65,
                'max_tf_age': 0.35,
                'odom_frame': 'odom_combined',
                'max_map_odom_age': 2.0,
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [racecar_launchr, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file,
                'use_composition': use_composition}.items(),
        ),

    ])
