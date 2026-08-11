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
import time  # 新增：用于执行停顿

class NavThroughPosesClient(Node):
    def __init__(self):
        super().__init__('nav_through_poses_client')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._goal_handle = None # 用于记录当前任务句柄

        self.marker_pub = self.create_publisher(Marker, "/forbidden_zone_marker", 10)
        self.timer = self.create_timer(1.0, self.show_forbidden_zone)

        # 新增：用于发布到达特定点时的文本
        self.text_pub = self.create_publisher(String, "/special_goal_topic", 10)

        # 新增：状态机，用于区分当前导航阶段
        # 0: 第一阶段(dating.csv), 1: 扫码后前半段, 2: 扫码后剩余段
        self.current_phase = 0
        self.part2_poses = []  # 暂存第二阶段剩余的未走路点

        # 二维码接收
        self.qr_result = None
        self.qr_sub = self.create_subscription(
            String,
            "/qr_result",
            self.qr_callback,
            10
        )

        self.get_logger().info("导航路点客户端已启动，等待 Nav2 连接...")
        self.get_logger().info("二维码监听已启动：识别到数字后将立即切换路径")

    # 强制接收，一旦收到有效数字，立即停止当前任务并执行下一阶段
    def qr_callback(self, msg):
        if self.qr_result is not None:  # 如果已经处理过结果，则跳过
            return

        text = msg.data
        self.get_logger().info(f"【回调触发】收到二维码原始数据：{text}")

        # 提取数字
        nums = re.findall(r'\d+', text)
        if nums:
            num = int(nums[0])
            self.qr_result = num
            self.get_logger().info(f"【触发切换】识别到数字 {self.qr_result}，立即中断当前任务！")
            
            # 如果当前正在导航，则取消当前任务，这会触发 get_result_callback
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            else:
                # 如果任务还没开始或已结束，直接手动调用逻辑
                self.execute_next_phase()

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
            self.get_logger().error("当前发送的所有路点都在禁区内！")
            # 即使全在禁区跳过，也应该触发行程结束回调以防死锁
            self.get_result_callback(None)
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

        self._goal_handle = goal_handle # 保存当前句柄
        self.get_logger().info("路点任务已接受！")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # 依靠 current_phase 状态机决定进入哪段逻辑
        if self.current_phase == 0:
            self.execute_next_phase()
        elif self.current_phase == 1:
            self.execute_part2()
        elif self.current_phase == 2:
            self.get_logger().info("所有阶段导航完成！")
            rclpy.shutdown()

    def execute_next_phase(self):
        # 无论是第一阶段正常走完，还是被二维码中断取消，都会触发到这里
        if self.qr_result is None:
            self.get_logger().info("dating.csv 跑完或被取消，等待二维码识别...")
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
            self.get_logger().info("奇数：执行 shunshuizhen1.csv，将在到达第 12 个点后停顿发布")
            next_wp = self.read_waypoints_from_csv("/root/ros2_ws/src/racecar/scripts/point/shunshizhen1.csv")
            split_idx = 9  # 顺时针第 6 个点作为切分点
        else:
            self.get_logger().info("偶数：执行 nishuizhen1.csv，将在到达第 12 个点后停顿发布")
            next_wp = self.read_waypoints_from_csv("/root/ros2_ws/src/racecar/scripts/point/test_1.csv")
            split_idx = 10  # 逆时针第 5 个点作为切分点

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

        # 切割路点
        part1_poses = next_poses[:split_idx]
        self.part2_poses = next_poses[split_idx:]
        
        # 阶段标记为1，发送前半段
        self.current_phase = 1
        self.send_goal(part1_poses)

    def execute_part2(self):
        # 前半段任务到达后执行此逻辑
        self.get_logger().info("已到达指定的中断点，停顿 0.1s 并发布文本...")
        
        # 发布文本 gaol
        msg = String()
        msg.data = "gaol"
        self.text_pub.publish(msg)

        # 停顿 0.1s
        time.sleep(0.0)

        self.get_logger().info("停顿结束，发送剩余的路点任务...")
        
        # 阶段标记为2，发送后半段（如果有的话）
        self.current_phase = 2
        if self.part2_poses:
            self.send_goal(self.part2_poses)
        else:
            self.get_logger().info("没有剩余路点需要执行，所有阶段导航完成！")
            rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('剩余距离：{0:.2f} m'.format(feedback.distance_remaining), once=True)

def main(args=None):
    rclpy.init(args=args)
    client = NavThroughPosesClient()

    waypoints = client.read_waypoints_from_csv("/root/ros2_ws/src/racecar/scripts/point/dating.csv")

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