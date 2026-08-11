#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
import csv
import os
import re

class NavThroughPosesClient(Node):
    def __init__(self):
        super().__init__('nav_through_poses_client')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.marker_pub = self.create_publisher(Marker, "/forbidden_zone_marker", 10)
        self.timer = self.create_timer(1.0, self.show_forbidden_zone)

        self.qr_result = None
        self.qr_sub = self.create_subscription(
            String,
            "/qr_result",
            self.qr_callback,
            10
        )

        # 代价地图相关的变量与订阅
        self.current_costmap = None
        self.initial_poses = []
        self.first_goal_sent = False

        # Nav2的全局代价地图通常使用 transient local 策略发布
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            qos
        )

        self.get_logger().info("导航路点客户端已启动，等待 Nav2 连接...")
        self.get_logger().info("禁区：X 2.35~4.3m | Y 1.9~2.4m")
        self.get_logger().info("已启用双重安全检查：固定禁区与全局代价地图")
        self.get_logger().info("二维码监听已启动：/qr_result (String类型)")

    def costmap_callback(self, msg):
        self.current_costmap = msg
        # 第一次接收到代价地图且已有初始路径点时，开始发送目标
        if not self.first_goal_sent and self.initial_poses:
            self.get_logger().info("成功接收到全局代价地图，开始进行路点安全筛查...")
            self.send_goal(self.initial_poses)
            self.first_goal_sent = True

    def qr_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"收到二维码原始数据：{text}")
        nums = re.findall(r'\d+', text)
        if nums:
            num = int(nums[0])
            self.qr_result = num
            self.get_logger().info(f"已保存二维码数字：{self.qr_result}")

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

    def is_in_costmap_obstacle(self, x, y):
        if self.current_costmap is None:
            self.get_logger().warn("警告：尚未接收到代价地图数据，跳过栅格碰撞检测。")
            return False

        resolution = self.current_costmap.info.resolution
        origin_x = self.current_costmap.info.origin.position.x
        origin_y = self.current_costmap.info.origin.position.y
        width = self.current_costmap.info.width
        height = self.current_costmap.info.height

        # 世界坐标 (x,y) 转换为 代价地图栅格坐标 (mx,my)
        mx = int((x - origin_x) / resolution)
        my = int((y - origin_y) / resolution)

        # 检查是否越界
        if mx < 0 or mx >= width or my < 0 or my >= height:
            self.get_logger().warn(f"坐标 ({x:.2f}, {y:.2f}) 超出代价地图边界，视为非法点。")
            return True

        index = my * width + mx
        cost = self.current_costmap.data[index]

        # 254: 致命障碍物 (Lethal Obstacle), 255: 未知区域 (Unknown)
        if cost == 254 or cost == 255:
            return True
        
        return False

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
            
            # 1. 检查硬禁区
            if self.is_in_forbidden_zone(x, y):
                self.get_logger().warn("路点 {0} 在硬禁区内，已跳过：({1:.2f}, {2:.2f})".format(i, x, y))
                continue
            
            # 2. 检查代价地图障碍物
            if self.is_in_costmap_obstacle(x, y):
                self.get_logger().warn("路点 {0} 位于障碍物或未知区域(Cost=254/255)，已跳过：({1:.2f}, {2:.2f})".format(i, x, y))
                continue

            safe_poses.append(pose)

        if not safe_poses:
            self.get_logger().error("所有路点都在禁区或障碍物内，导航中止！")
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
        self.get_logger().info("当前 CSV 路点导航完成！")

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
            self.get_logger().info("奇数：执行 shunshizhen1.csv")
            next_wp = self.read_waypoints_from_csv("/root/ros2_ws/src/racecar/scripts/shunshizhen1.csv")
        else:
            self.get_logger().info("偶数：执行 nishizhen4.csv")
            next_wp = self.read_waypoints_from_csv("/root/ros2_ws/src/racecar/scripts/nishizhen4.1.csv")

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

    # 不再立即发送路点，而是存入列表等待地图回调触发
    client.initial_poses = poses
    client.get_logger().info("已读取初始路点，等待获取全局代价地图...")
    
    rclpy.spin(client)

if __name__ == '__main__':
    main()