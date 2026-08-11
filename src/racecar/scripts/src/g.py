#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String  # 必须是String！
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import csv
import os
import re

class NavThroughPosesClient(Node):
    def __init__(self):
        super().__init__('nav_through_poses_client')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.marker_pub = self.create_publisher(Marker, "/forbidden_zone_marker", 10)
        self.timer = self.create_timer(1.0, self.show_forbidden_zone)

        # 二维码接收（强制开启，无任何限制！）
        self.qr_result = None
        self.qr_sub = self.create_subscription(
            String,
            "/qr_result",
            self.qr_callback,
            10
        )

        self.get_logger().info("导航路点客户端已启动，等待 Nav2 连接...")
        self.get_logger().info("禁区：X 2.35~4.3m | Y 1.9~2.4m")
        self.get_logger().info("已启用路径安全检查：小车会自动避开禁区！")
        self.get_logger().info("✅ 二维码监听已启动：/qr_result (String类型)")

    # 强制接收，无任何过滤！！！！
    def qr_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"【回调触发】收到二维码原始数据：{text}")

        # 提取数字
        nums = re.findall(r'\d+', text)
        if nums:
            num = int(nums[0])
            self.qr_result = num
            self.get_logger().info(f"【已保存】二维码数字：{self.qr_result}")

    def show_forbidden_zone(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "forbidden_zone"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = (2.35 + 4.3) / 2
        marker.pose.position.y = (1.9 + 2.4) / 2
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 4.3 - 2.35
        marker.scale.y = 2.4 - 1.9
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.3
        self.marker_pub.publish(marker)

    def is_in_forbidden_zone(self, x, y):
        return 2.35 <= x <= 4.3 and 1.9 <= y <= 2.4

    def read_waypoints_from_csv(self, file_path):
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

    def send_goal(self, poses):
        safe_poses = []
        for i, pose in enumerate(poses):
            x = pose.pose.position.x
            y = pose.pose.position.y
            if self.is_in_forbidden_zone(x, y):
                self.get_logger().warn("路点 {0} 在禁区内，已跳过：({1:.2f}, {2:.2f})".format(i, x, y))
                continue
            safe_poses.append(pose)

        if not safe_poses:
            self.get_logger().error("所有路点都在禁区内！")
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = safe_poses

        self._action_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info("发送安全路点，共 {0} 个".format(len(safe_poses)))
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("路点任务被拒绝！")
            return

        self.get_logger().info("路点任务已接受！")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info("dating.csv 路点导航完成！")

        # 等待确保收到（最长等3秒）
        for i in range(30):
            if self.qr_result is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.qr_result is None:
            self.get_logger().error("未收到二维码，导航结束")
            rclpy.shutdown()
            return

        self.get_logger().info(f"使用二维码结果：{self.qr_result}")

        if self.qr_result % 2 == 1:
            self.get_logger().info("奇数：执行 shunshuizhen1.csv")
            next_wp = self.read_waypoints_from_csv("/root/ros2_ws/src/racecar/scripts/shunshizhen1.csv")
        else:
            self.get_logger().info("偶数：执行 nishuizhen1.csv")
            next_wp = self.read_waypoints_from_csv("/root/ros2_ws/src/racecar/scripts/nishizhen4.csv")

        next_poses = []
        for x, y, z, w in next_wp:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = z
            pose.pose.orientation.w = w
            next_poses.append(pose)

        self.send_goal(next_poses)

        self.get_result_callback = lambda f: (
            self.get_logger().info("所有路点导航完成！"),
            rclpy.shutdown()
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('剩余距离：{0:.2f} m'.format(feedback.distance_remaining))

def main(args=None):
    rclpy.init(args=args)
    client = NavThroughPosesClient()

    waypoints = client.read_waypoints_from_csv("/root/ros2_ws/src/racecar/scripts/dating.csv")

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