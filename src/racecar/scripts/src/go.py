#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
import csv

class NavThroughPosesClient(Node):
    def __init__(self):
        super().__init__('nav_through_poses_client')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.get_logger().info(" 导航路点客户端已启动，等待 Nav2 连接...")

    def send_goal(self, poses):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        # 等待导航服务上线
        self._action_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info(f" 发送路点任务，共 {len(poses)} 个点")
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(" 路点任务被拒绝！")
            return

        self.get_logger().info(" 路点任务已接受！")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(" 所有路点导航完成！")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f' 剩余距离：{feedback.distance_remaining:.2f} m')

def read_waypoints_from_csv(file_path):
    """从 CSV 读取路点：x,y,z_orientation,w_orientation"""
    waypoints = []
    with open(file_path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) == 4:
                x = float(row[0])
                y = float(row[1])
                z = float(row[2])
                w = float(row[3])
                waypoints.append((x, y, z, w))
    return waypoints

def main(args=None):
    rclpy.init(args=args)
    client = NavThroughPosesClient()

    # ====================== 你的路点文件路径 ======================
    csv_path = "/root/ros2_ws/src/racecar/scripts/waypoints.csv"
    # =============================================================

    waypoints = read_waypoints_from_csv(csv_path)

    # 转成 ROS2 位姿格式
    poses = []
    for x, y, z, w in waypoints:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = client.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        poses.append(pose)

    client.send_goal(poses)
    rclpy.spin(client)

if __name__ == '__main__':
    main()