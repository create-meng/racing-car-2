#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import csv
import os
import re

class GlobalNavClient(Node):
    def __init__(self):
        super().__init__('global_nav_client')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.marker_pub = self.create_publisher(Marker, "/forbidden_zone_marker", 10)
        self.timer = self.create_timer(1.0, self.show_forbidden_zone)

        # 二维码接收
        self.qr_result = None
        self.qr_sub = self.create_subscription(
            String,
            "/qr_result",
            self.qr_callback,
            10
        )

        self.is_second_stage = False  # 记录是否处于第二阶段

        self.get_logger().info(" 全局预判导航客户端启动！")
        self.get_logger().info("模式：NavigateThroughPoses (整体路径规划 + 遇障自动结束)")
        self.get_logger().info("禁区：X 2.35~4.3m | Y 1.9~2.4m")

    def qr_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"【回调触发】收到二维码原始数据：{text}")
        nums = re.findall(r'\d+', text)
        if nums:
            self.qr_result = int(nums[0])
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
                self.get_logger().warn("路点在禁区内，已跳过：({0:.2f}, {1:.2f})".format(x, y))
                continue
            safe_poses.append(pose)

        if not safe_poses:
            self.get_logger().info("无有效路点，直接进入下一步。")
            if not self.is_second_stage:
                self.handle_second_stage()
            else:
                rclpy.shutdown()
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = safe_poses

        self._action_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info(f"发送全局路径，共 {len(safe_poses)} 个点")
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("️ 目标被Nav2直接拒绝！强制进入下一阶段...")
            if not self.is_second_stage:
                self.handle_second_stage()
            else:
                rclpy.shutdown()
            return

        self.get_logger().info("路径已被接受，开始执行...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        # 【核心修复】：无论状态是 4(成功) 还是其他(受阻失败)，都不再回头重试，直接当做跑完！
        if status == 4:
            self.get_logger().info(" 本阶段导航完美抵达终点！")
        else:
            self.get_logger().warn(f"️ 导航最终因障碍物停止 (状态码:{status})。强制视为本阶段结束！")

        if not self.is_second_stage:
            self.handle_second_stage()
        else:
            self.get_logger().info(" 第二阶段结束，系统关闭！")
            rclpy.shutdown()

    def handle_second_stage(self):
        self.is_second_stage = True
        self.get_logger().info("等待二维码判定分叉路径...")
        
        # 等待确保收到二维码（最长等3秒）
        for i in range(30):
            if self.qr_result is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.qr_result is None:
            self.get_logger().error("未收到二维码，导航结束")
            rclpy.shutdown()
            return

        self.get_logger().info(f"使用二维码结果：{self.qr_result}")

        # 根据奇偶选择 CSV
        if self.qr_result % 2 == 1:
            self.get_logger().info("奇数：执行 shunshizhen1.csv")
            next_wp = self.read_waypoints_from_csv("/root/ros2_ws/src/racecar/scripts/shunshizhen1.csv")
        else:
            self.get_logger().info("偶数：执行 nishizhen1.csv")
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

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('总剩余距离：{0:.2f} m'.format(feedback_msg.feedback.distance_remaining))

def main(args=None):
    rclpy.init(args=args)
    client = GlobalNavClient()

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