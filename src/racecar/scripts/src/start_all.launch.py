from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 先启动：导航路点节点
    nav_node = ExecuteProcess(
        cmd=['python3', '/root/ros2_ws/src/racecar/scripts/src/go4.py'],
        output='screen'
    )

    # 2. 后启动：巡线节点（等导航节点结束后才运行）
    line_follower_node = ExecuteProcess(
        cmd=['python3', '/userdata/dev_ws/src/origincar/line_follower_pkg/line_follower_pkg/line_follower_node.py'],
        output='screen'
    )

    # 关键：导航结束 → 自动启动巡线
    load_line_follower = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=nav_node,
            on_exit=[line_follower_node]
        )
    )

    ld = LaunchDescription()
    ld.add_action(nav_node)
    ld.add_action(load_line_follower)

    return ld