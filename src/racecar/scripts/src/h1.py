#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String  # 必须是String！
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from visualization_msgs.msg import Marker
import csv
import os
import re
import math

# ===== 扫码失败兜底方向（改这里即可，无需额外命令行参数）=====
# 二维码没扫到时，按此方向继续跑，不再停车：
#   1 = 顺时针（shunshizhen1.csv）
#   2 = 逆时针（test_1.csv）
FALLBACK_DIRECTION = 1


class NavThroughPosesClient(Node):
    def __init__(self):
        super().__init__('nav_through_poses_client')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._goal_handle = None # 用于记录当前任务句柄

        # 订阅 AMCL 定位，用于记录实际位姿到日志
        self.amcl_pose = None
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            10,
        )

        self.marker_pub = self.create_publisher(Marker, "/forbidden_zone_marker", 10)
        self.timer = self.create_timer(1.0, self.show_forbidden_zone)

        # 用于发布到达特定点时的文本
        self.text_pub = self.create_publisher(String, "/special_goal_topic", 10)

        # 用于语音播报方向（/tts_text 由 voice_broadcast_node 播出）
        self.tts_pub = self.create_publisher(String, "/tts_text", 10)

        # 拍照触发标志与路点统计
        self.photo_triggered = False  # 图生文已触发标志（仅触发一次）
        self.total_poses = 0
        self.first_phase_done = False  # 标记第一阶段(dating.csv)是否完成

        # 循环方向：1=顺时针 2=逆时针（扫码成功按奇偶定，失败按顶部 FALLBACK_DIRECTION 定）
        self.loop_direction = None

        # 第二阶段(循环)状态：步骤A 先到第一个路点回正航向，步骤B 再正着跑完整圈
        self.loop_poses = None      # 循环全量路点 (PoseStamped 列表)
        self.loop_total = 0         # 循环总路点数（步骤B 用于拍照计数）
        self.phase2_step = 0        # 0=步骤A(回正航向,单首点) 1=步骤B(整圈)
        self.phase2_retries = 0     # 步骤B 失败自动重试计数（临时 lethal 误判时继续走）
        self._last_visited = 0      # 最近一次到达的路点计数（用于失败后从中断处继续）

        # --- 卡点看门狗：5s 无进展就主动取消目标，跳过当前路点继续走，不等 Nav2 磨蹭 ---
        self._goal_active = False            # 当前是否有执行中的目标
        self._last_progress = self.get_clock().now()   # 最近一次"有进展"的时刻
        self._last_remaining = float('inf')  # 上一次反馈的剩余距离
        self.watchdog_timer = self.create_timer(1.0, self.watchdog_tick)

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

    def amcl_callback(self, msg):
        """记录 AMCL 定位位姿 (map 系)，并检查是否满足图生文触发条件。"""
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.amcl_pose = (p.position.x, p.position.y, yaw)
        self.check_amcl_photo_trigger()

    def check_amcl_photo_trigger(self):
        """图生文触发：AMCL 位姿进入允许区域时触发一次（仅触发一次）。

        允许触发区域（由循环方向决定，二维码或兜底参数）：
        - 顺时针循环：x∈[3.7,4.3]，y∈[3.7,4.3]
        - 逆时针循环：x∈[0.7,1.3]，y∈[3.7,4.3]
        """
        if self.photo_triggered:      # 已触发过 → 不再触发
            return
        if self.phase2_step != 1:     # 仅在整圈循环（步骤B）阶段触发
            return
        if self.amcl_pose is None or self.loop_direction is None:
            return

        x, y, _ = self.amcl_pose
        if self.loop_direction == 1:
            # 顺时针循环
            allowed = (3.7 <= x <= 4.3) and (3.7 <= y <= 4.3)
            region = "顺时针(3.7-4.3, 3.7-4.3)"
        else:
            # 逆时针循环
            allowed = (0.7 <= x <= 1.3) and (3.7 <= y <= 4.3)
            region = "逆时针(0.7-1.3, 3.7-4.3)"

        if allowed:
            msg = String()
            msg.data = "gaol"
            self.text_pub.publish(msg)
            self.photo_triggered = True
            self.get_logger().info(
                f"【图生文触发】AMCL 位姿({x:.3f}, {y:.3f}) 进入 {region}，已触发（仅一次）")

    def log_amcl(self, tag):
        """把当前 AMCL 位姿写入日志。"""
        if self.amcl_pose is None:
            self.get_logger().info(f"[{tag}] amcl_pose: (无位姿)")
            return
        x, y, yaw = self.amcl_pose
        self.get_logger().info(
            f"[{tag}] amcl_pose: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}°"
        )

    # 二维码接收
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

    def build_poses(self, wp_list):
        """把 (x,y,z,w) 四元组列表构造成 PoseStamped 列表。"""
        poses = []
        for x, y, z, w in wp_list:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = z
            pose.pose.orientation.w = w
            poses.append(pose)
        return poses

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
            self.get_logger().error("路点任务被拒绝！按失败推进状态机，避免卡死")
            self.get_result_callback(None)
            return

        self._goal_handle = goal_handle # 保存当前句柄
        self.get_logger().info("路点任务已接受！")
        self.log_amcl("任务开始")
        # 重置看门狗计时
        self._goal_active = True
        self._last_progress = self.get_clock().now()
        self._last_remaining = float('inf')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self._goal_active = False  # 看门狗：当前目标已结束
        # 第一阶段(dating.csv)结束或被扫码中断 → 进入第二阶段
        if not self.first_phase_done:
            self.first_phase_done = True
            self.execute_next_phase()
            return

        # ---- 第二阶段状态机 ----
        status = None
        if future is not None and future.result() is not None:
            status = future.result().status  # goal_status: 4=SUCCEEDED

        if self.phase2_step == 0:
            # 步骤A（先到第一个路点回正航向）结束
            if status == 4:
                self.get_logger().info("已到第一个路点，航向回正，开始正着跑完整圈")
                self.phase2_step = 1
                self.phase2_retries = 0
                self._last_visited = 1
                self.total_poses = self.loop_total
                self.photo_triggered = False
                self.send_goal(self.loop_poses[1:])
            else:
                # 失败/被取消/空目标 → 回退为整圈全量路点方式，保证任务不卡死
                self.get_logger().warn(
                    f"步骤A(回正航向)未成功(status={status})，回退为整圈全量路点方式")
                self.phase2_step = 1
                self.total_poses = self.loop_total
                self.photo_triggered = False
                self.send_goal(self.loop_poses)
        else:
            # 步骤B（整圈）结束
            if status == 4:
                self.get_logger().info("所有路点导航完成！")
                rclpy.shutdown()
            else:
                # 卡墙/取消 → 跳过当前不可达路点，继续下一个
                start = max(1, self._last_visited) + 1
                if start >= self.loop_total:
                    self.get_logger().info("所有路点已完成！")
                    rclpy.shutdown()
                    return
                self._last_visited = start
                self.get_logger().warn(
                    f"卡墙，跳过第 {start-1} 个路点，从第 {start} 个继续走（共 {self.loop_total} 点）")
                self.total_poses = self.loop_total
                # 注意：卡墙重试不重置 photo_triggered → 图生文整个任务仅触发一次
                # 短暂等待后重新规划
                for _ in range(5):
                    rclpy.spin_once(self, timeout_sec=0.1)
                self.send_goal(self.loop_poses[start:])

    def publish_tts(self, text):
        """发布语音播报文本到 /tts_text（由 voice_broadcast_node 播出）。"""
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)
        self.get_logger().info(f"语音播报: {text}")

    def execute_next_phase(self):
        # 第一阶段结束：先等二维码（最长 7 秒）
        if self.qr_result is None:
            self.get_logger().info("dating.csv 跑完或被取消，等待二维码识别（最长 7 秒）...")
            for i in range(70):
                if self.qr_result is not None:
                    break
                rclpy.spin_once(self, timeout_sec=0.1)

        if self.qr_result is not None:
            # ---- 扫码成功：奇数=顺时针，偶数=逆时针；播报 数字+方向 ----
            self.loop_direction = 1 if self.qr_result % 2 == 1 else 2
            dir_name = "顺时针" if self.loop_direction == 1 else "逆时针"
            self.get_logger().info(f"使用二维码结果：{self.qr_result} → {dir_name}")
            self.publish_tts(f"{self.qr_result}，{dir_name}")
        else:
            # ---- 扫码失败：按 h1.py 顶部 FALLBACK_DIRECTION 兜底；只播报方向 ----
            self.loop_direction = FALLBACK_DIRECTION
            dir_name = "顺时针" if self.loop_direction == 1 else "逆时针"
            self.get_logger().warn(f"未收到二维码，按顶部参数兜底执行：{dir_name}")
            self.publish_tts(dir_name)

        if self.loop_direction == 1:
            self.get_logger().info("执行 shunshizhen1.csv（顺时针循环）")
            next_wp = self.read_waypoints_from_csv("/root/ros2_ws/src/racecar/scripts/point/shunshizhen1.csv")
        else:
            self.get_logger().info("执行 test_1.csv（逆时针循环）")
            next_wp = self.read_waypoints_from_csv("/root/ros2_ws/src/racecar/scripts/point/test_1.csv")

        # 图生文触发：由 AMCL 位姿区域触发（见 check_amcl_photo_trigger），
        # 不再按路点计数触发。这里只构建循环全量路点。
        self.loop_poses = self.build_poses(next_wp)
        self.loop_total = len(self.loop_poses)

        # 步骤A：先只发第一个路点，强制到点后回正航向（对准循环前进方向）
        # 步骤A期间 total_poses=1 且抑制拍照，避免误触发
        self.phase2_step = 0
        self.photo_triggered = True
        self.total_poses = 1
        self.get_logger().info(f"步骤A：先导航到循环第一个路点以回正航向（共 {self.loop_total} 个点）")
        self.send_goal(self.loop_poses[:1])

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = feedback.distance_remaining
        # 看门狗：剩余距离下降 = 有进展，刷新计时
        if remaining < self._last_remaining:
            self._last_progress = self.get_clock().now()
        self._last_remaining = remaining
        if self.total_poses > 0:
            visited = self.total_poses - feedback.number_of_poses_remaining
            self.get_logger().info(
                "剩余距离：{0:.2f} m | 已过 {1}/{2} 个点".format(remaining, visited, self.total_poses),
                once=True,
            )
            # 每到达一个新路点时记录 AMCL 位姿
            if hasattr(self, '_last_visited') and visited != self._last_visited:
                self.log_amcl(f"到达路点 {visited}")
            self._last_visited = visited
        else:
            self.get_logger().info(
                "剩余距离：{0:.2f} m | 剩余路点 {1} 个".format(remaining, feedback.number_of_poses_remaining),
                once=True,
            )

        # 图生文触发已改为 AMCL 位姿区域触发（check_amcl_photo_trigger），此处不再触发

    def watchdog_tick(self):
        """1Hz 看门狗：目标在执行但 5s 无进展 → 主动取消，跳过该路点继续走。"""
        if not self._goal_active or self._goal_handle is None:
            return
        if (self.get_clock().now() - self._last_progress).nanoseconds > 5e9:
            self.get_logger().warn("看门狗：5s 无进展，主动取消当前目标以继续走")
            self._goal_handle.cancel_goal_async()
            self._last_progress = self.get_clock().now()

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