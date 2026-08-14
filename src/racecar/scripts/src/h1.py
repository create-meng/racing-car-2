#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String  # 必须是String！
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
import csv
import os
import re
import math


class NavThroughPosesClient(Node):
    FINAL_RETURN_X = 0.0
    FINAL_RETURN_Y = 0.1
    POINT_REACHED_RADIUS = 0.30
    ESCAPE_COST = 254
    ESCAPE_CONFIRMATIONS = 3
    ESCAPE_SPEED = -0.05
    ESCAPE_DISTANCE = 0.15
    ESCAPE_REAR_CLEARANCE = 0.30
    ESCAPE_FRONT_CLEARANCE = 0.20
    FIRST_PHASE_RETRY_DELAY = 1.0
    QR_POINT_TIMEOUT = 20.0

    def __init__(self):
        super().__init__('nav_through_poses_client')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._goal_handle = None # 用于记录当前任务句柄
        self._goal_pending = False  # 首阶段目标已发送，但服务端尚未返回句柄
        self._first_phase_cancel_requested = False
        self._qr_route_poses = []
        self._qr_route_offset = 0
        self._qr_replan_requested = False
        self._qr_last_progress_time = None
        self._qr_last_amcl = None
        self._qr_last_remaining = None
        self._qr_target_index = None
        self._qr_target_started_at = None
        self._qr_stall_timeout = 12.0
        self._qr_escape_distance = 0.22
        self._qr_escape_timeout = 5.0
        self._qr_escape_timer = None
        self._qr_escape_started_at = None
        self._qr_escape_start_amcl = None
        self._qr_escape_turn = 0.0
        self._escape_speed = 0.0
        self._escape_preferred_turn = 0.0
        self._qr_escape_replan_pending = False
        self._qr_escape_attempts = {}
        self._costmap_escape_hits = 0
        self._escape_requires_clear = False
        self._local_costmap = None
        self._latest_scan = None
        self._escape_wait_logged_at = None
        self._active_route_poses = []
        self._active_route_offset = 0
        self._dating_route_poses = []
        self._first_phase_retry_timer = None

        # 订阅 AMCL 定位，用于记录实际位姿到日志
        self.amcl_pose = None
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.local_costmap_sub = self.create_subscription(
            Costmap, "/local_costmap/costmap_raw", self.local_costmap_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )

        self.marker_pub = self.create_publisher(Marker, "/forbidden_zone_marker", 10)
        self.timer = self.create_timer(1.0, self.show_forbidden_zone)
        self.route_monitor_timer = self.create_timer(1.0, self.monitor_route)

        # 用于发布到达特定点时的文本
        self.text_pub = self.create_publisher(String, "/special_goal_topic", 10)
        # Nav2 velocity_smoother 的输入；支持倒车时同时提供角速度。
        self.escape_cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)

        # 拍照触发标志与路点统计
        self.photo_triggered = False
        self.total_poses = 0
        self._last_visited = None
        self.first_phase_done = False
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
        """记录 AMCL 定位位姿 (map 系)。"""
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.amcl_pose = (p.position.x, p.position.y, yaw)

    def scan_callback(self, msg):
        self._latest_scan = msg

    def local_costmap_pose(self, costmap):
        try:
            transform = self.tf_buffer.lookup_transform(
                costmap.header.frame_id, "base_footprint", Time()
            )
        except TransformException:
            return None

        position = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return position.x, position.y, yaw

    @staticmethod
    def footprint_costs_at(costmap, x, y, yaw):
        info = costmap.metadata
        costs = []
        for px in (-0.125, -0.075, -0.025, 0.025, 0.075, 0.125):
            for py in (-0.095, -0.045, 0.005, 0.055, 0.105):
                fx = x + px * math.cos(yaw) - py * math.sin(yaw)
                fy = y + px * math.sin(yaw) + py * math.cos(yaw)
                cx = int((fx - info.origin.position.x) / info.resolution)
                cy = int((fy - info.origin.position.y) / info.resolution)
                if 0 <= cx < info.size_x and 0 <= cy < info.size_y:
                    costs.append(costmap.data[cy * info.size_x + cx])
        return costs

    def local_footprint_cost(self, costmap):
        """返回局部代价地图中当前车体 footprint 的最大原始代价。"""
        pose = self.local_costmap_pose(costmap)
        if pose is None:
            return None
        costs = self.footprint_costs_at(costmap, *pose)
        return max(costs) if costs else None

    def local_costmap_callback(self, costmap):
        """二维码路线持续进入 lethal 区时，取消导航后选择安全脱困动作。"""
        self._local_costmap = costmap
        if not self.first_phase_done or self.qr_result is None:
            self._costmap_escape_hits = 0
            self._escape_requires_clear = False
            return

        cost = self.local_footprint_cost(costmap)
        if cost is None or cost < self.ESCAPE_COST:
            self._costmap_escape_hits = 0
            self._escape_requires_clear = False
            return

        if (
            self._escape_requires_clear
            or self._goal_handle is None
            or self._goal_pending
            or self._qr_replan_requested
            or self._qr_escape_timer is not None
            or self._first_phase_cancel_requested
        ):
            self._costmap_escape_hits = 0
            return

        self._costmap_escape_hits += 1
        if self._costmap_escape_hits < self.ESCAPE_CONFIRMATIONS:
            return

        self._escape_requires_clear = True
        self.request_qr_escape(
            f"footprint 连续 {self._costmap_escape_hits} 帧进入 lethal 代价 {cost}"
        )

    def scan_clearance(self, forward):
        if self._latest_scan is None:
            return None
        stamp = self._latest_scan.header.stamp
        age = (self.get_clock().now().nanoseconds - stamp.sec * 1_000_000_000 - stamp.nanosec) / 1e9
        if age > 0.35:
            return None
        ranges = []
        for i, distance in enumerate(self._latest_scan.ranges):
            if not math.isfinite(distance) or distance <= 0.05:
                continue
            angle = self._latest_scan.angle_min + i * self._latest_scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            in_direction = abs(angle) <= math.pi / 3 if forward else abs(abs(angle) - math.pi) <= math.pi / 3
            if in_direction:
                ranges.append(distance)
        return min(ranges) if ranges else None

    def escape_path_is_clear(self, speed, turn):
        """检查前进或倒车弧线未来 0.15m 的车体扫掠区。"""
        if self._local_costmap is None:
            return False, "无 local_costmap"
        forward = speed > 0.0
        clearance = self.scan_clearance(forward)
        required_clearance = self.ESCAPE_FRONT_CLEARANCE if forward else self.ESCAPE_REAR_CLEARANCE
        direction = "前向" if forward else "后向"
        if clearance is None or clearance < required_clearance:
            value = f"无新鲜{direction}激光" if clearance is None else f"{direction}仅 {clearance:.2f}m"
            return False, value

        pose = self.local_costmap_pose(self._local_costmap)
        if pose is None:
            return False, "无 base_footprint TF"
        x, y, yaw = pose
        current_costs = self.footprint_costs_at(self._local_costmap, x, y, yaw)
        current_inscribed = sum(cost >= self.ESCAPE_COST for cost in current_costs)
        step_distance = 0.025
        for _ in range(int(self.ESCAPE_DISTANCE / step_distance)):
            dt = step_distance / abs(speed)
            x += speed * math.cos(yaw) * dt
            y += speed * math.sin(yaw) * dt
            yaw += turn * dt
            costs = self.footprint_costs_at(self._local_costmap, x, y, yaw)
            if not costs or max(costs) >= 254:
                return False, f"{direction}弧线进入 lethal"
            if sum(cost >= self.ESCAPE_COST for cost in costs) > current_inscribed:
                return False, f"{direction}弧线更深入障碍"
        return True, ""

    def choose_escape_motion(self, preferred_turn):
        alternatives = (
            (self.ESCAPE_SPEED, preferred_turn),
            (self.ESCAPE_SPEED, -preferred_turn),
            (-self.ESCAPE_SPEED, preferred_turn),
            (-self.ESCAPE_SPEED, -preferred_turn),
        )
        reasons = []
        for speed, turn in alternatives:
            clear, reason = self.escape_path_is_clear(speed, turn)
            if clear:
                return speed, turn, ""
            reasons.append(reason)
        return None, None, "；".join(reasons)

    def request_qr_escape(self, reason):
        """二维码路线的所有脱困触发统一取消任务，再进入同一脱困状态。"""
        if not self.first_phase_done or self.qr_result is None or self._qr_replan_requested:
            return
        self._qr_replan_requested = True
        self._qr_escape_replan_pending = True
        self.get_logger().warn(f"[脱困] {reason}，取消当前导航并选择安全动作")
        if self._goal_handle is not None and not self._goal_pending:
            self._goal_handle.cancel_goal_async().add_done_callback(self.cancel_response_callback)
        else:
            self._qr_escape_replan_pending = False
            self.start_qr_escape()

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
            self.get_logger().info(f"【收到二维码】识别到数字 {self.qr_result}")
            if self._goal_handle is not None and not self.first_phase_done:
                self.get_logger().info("正在执行 dating.csv，取消当前任务并切换二维码路线")
                self.cancel_first_phase_goal()
            elif self._goal_pending and not self.first_phase_done:
                self.get_logger().info("首阶段目标仍在等待 Nav2 接受，接受后立即取消并切换二维码路线")
            elif self.first_phase_done:
                self.execute_next_phase()
            else:
                # 首阶段重试计时期间没有活动 action；扫码仍应立即进入下一阶段。
                self.first_phase_done = True
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

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("navigate_through_poses 服务未就绪，未发送首阶段目标")
            return
        self.get_logger().info("发送安全路点，共 {0} 个".format(len(safe_poses)))
        self._goal_pending = True
        self._first_phase_cancel_requested = False
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def route_poses(self):
        return self._qr_route_poses if self.first_phase_done else self._dating_route_poses

    def passed_route_prefix(self, route, start_index, x, y):
        """只删除进入通过半径的连续路点，不能因绕远而跳过目标。"""
        index = max(0, start_index)
        while index < len(route) - 1:
            point = route[index].pose.position
            distance = math.hypot(x - point.x, y - point.y)
            if distance > self.POINT_REACHED_RADIUS:
                break
            index += 1
        return index

    def monitor_route(self):
        """全程打印 AMCL 与当前目标；二维码路线额外处理无进展重发。"""
        if not self._active_route_poses:
            return
        if self._goal_handle is None or self._goal_pending:
            return

        local_target = self._last_visited if self._last_visited is not None else 0
        local_target = min(local_target, len(self._active_route_poses) - 1)
        target_index = self._active_route_offset + local_target
        target = self._active_route_poses[local_target].pose.position
        phase = "二维码路线" if self.first_phase_done else "dating.csv"
        route_count = len(self._qr_route_poses) if self.first_phase_done else len(self._active_route_poses)
        if self.amcl_pose is None:
            self.get_logger().warn(
                f"[追点][{phase}] 目标[{target_index + 1}/{route_count}] "
                f"x={target.x:.3f} y={target.y:.3f} | AMCL 无位姿"
            )
            return

        amcl_x, amcl_y, _ = self.amcl_pose
        distance = math.hypot(target.x - amcl_x, target.y - amcl_y)
        self.get_logger().info(
            f"[追点][{phase}] AMCL x={amcl_x:.3f} y={amcl_y:.3f} | "
            f"目标[{target_index + 1}/{route_count}] "
            f"x={target.x:.3f} y={target.y:.3f} | 距离={distance:.3f} m"
        )
        if not self.first_phase_done:
            return

        # 大删除：仅删除连续进入通过半径的点，绕远时仍追原目标。
        route = self.route_poses()
        passed_to = self.passed_route_prefix(
            route, self._active_route_offset, amcl_x, amcl_y
        )
        if passed_to > self._active_route_offset and passed_to < len(route):
            if not self._qr_replan_requested:
                old_offset = self._active_route_offset
                self._qr_route_offset = passed_to
                self._qr_replan_requested = True
                self._qr_escape_replan_pending = False
                self.get_logger().warn(
                    f"[追点] 已越过路点[{old_offset + 1}~{passed_to}]，批量删除；"
                    f"下一目标[{passed_to + 1}]，AMCL=({amcl_x:.3f},{amcl_y:.3f})"
                )
                self._goal_handle.cancel_goal_async().add_done_callback(
                    self.cancel_response_callback
                )
                return

        now = self.get_clock().now().nanoseconds / 1e9
        if self._qr_target_index != target_index:
            self._qr_target_index = target_index
            self._qr_target_started_at = now
        elif now - self._qr_target_started_at >= self.QR_POINT_TIMEOUT:
            if self._qr_replan_requested:
                return
            if target_index < len(self.route_poses()) - 1:
                self._qr_route_offset = target_index + 1
                self.request_qr_escape(
                    f"目标[{target_index + 1}] 已追踪 {self.QR_POINT_TIMEOUT:.0f}s，"
                    f"无论是否仍有位移均跳过，脱困后从目标[{self._qr_route_offset + 1}]继续"
                )
            else:
                self.request_qr_escape(
                    f"最终目标已追踪 {self.QR_POINT_TIMEOUT:.0f}s，禁止跳过，仅脱困后重试"
                )
            return

        moved = self._qr_last_amcl is None or math.hypot(
            amcl_x - self._qr_last_amcl[0], amcl_y - self._qr_last_amcl[1]
        ) >= 0.03
        remaining = self._qr_last_remaining
        if moved or remaining != self._last_visited:
            self._qr_last_progress_time = now
            self._qr_last_amcl = (amcl_x, amcl_y)
            self._qr_last_remaining = self._last_visited
            return
        if self._qr_last_progress_time is None:
            self._qr_last_progress_time = now
            self._qr_last_amcl = (amcl_x, amcl_y)
            return

        if now - self._qr_last_progress_time < self._qr_stall_timeout:
            return
        if self._qr_replan_requested:
            return

        route_poses = self.route_poses()
        if target_index < len(route_poses) - 1:
            self._qr_route_offset = target_index + 1
            reason = (
                f"{self._qr_stall_timeout:.0f}s 无位移，跳过目标[{target_index + 1}]，"
                f"脱困后从目标[{self._qr_route_offset + 1}]继续"
            )
        else:
            reason = "最终回点无位移，禁止跳过，仅脱困后重试最终目标"
        self.request_qr_escape(reason)

    def prepare_qr_escape(self):
        """基于当前 AMCL 统一初始化目标朝向与首个安全脱困动作。"""
        if self.amcl_pose is None:
            return False
        local_target = min(
            self._last_visited if self._last_visited is not None else 0,
            len(self._active_route_poses) - 1,
        )
        target = self._active_route_poses[local_target].pose.position
        x, y, yaw = self.amcl_pose
        target_heading = math.atan2(target.y - y, target.x - x)
        angle_error = math.atan2(
            math.sin(target_heading - yaw), math.cos(target_heading - yaw)
        )
        self._qr_escape_turn = max(-0.8, min(0.8, 1.2 * angle_error))
        if abs(self._qr_escape_turn) < 0.25:
            self._qr_escape_turn = 0.25 if angle_error >= 0.0 else -0.25
        self._escape_preferred_turn = self._qr_escape_turn
        self._qr_escape_start_amcl = (x, y)
        selected_speed, selected_turn, reason = self.choose_escape_motion(
            self._escape_preferred_turn
        )
        if selected_speed is None:
            self._escape_speed = 0.0
            self._qr_escape_turn = 0.0
            self._escape_wait_logged_at = self._qr_escape_started_at
            self.get_logger().warn(f"[脱困] 前后脱困弧线都不安全，停车等待：{reason}")
        else:
            self._escape_speed = selected_speed
            self._qr_escape_turn = selected_turn
        motion = (
            "前进转向" if self._escape_speed > 0.0
            else "倒车转向" if self._escape_speed < 0.0
            else "停车等待"
        )
        self.get_logger().warn(
            f"[脱困] {motion}开始：目标角误差={math.degrees(angle_error):.1f}°，"
            f"linear.x={self._escape_speed:.2f} angular.z={self._qr_escape_turn:.2f}，"
            f"目标距离={self._qr_escape_distance:.2f}m"
        )
        return True

    def start_qr_escape(self):
        """二维码路线脱困：后方安全时优先倒车，否则使用安全的前进转弯。"""
        if self._qr_escape_timer is not None:
            return
        self._qr_escape_started_at = self.get_clock().now().nanoseconds / 1e9
        if not self.prepare_qr_escape():
            self.escape_cmd_pub.publish(Twist())
            self._escape_wait_logged_at = self._qr_escape_started_at
            self.get_logger().warn("[脱困] 等待 AMCL 位姿，保持停车，不重发路线")
        self._qr_escape_timer = self.create_timer(0.05, self.qr_escape_tick)

    def qr_escape_tick(self):
        if self._qr_escape_started_at is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.amcl_pose is None:
            self.escape_cmd_pub.publish(Twist())
            if self._escape_wait_logged_at is None or now - self._escape_wait_logged_at >= 1.0:
                self.get_logger().warn("[脱困] AMCL 位姿未恢复，保持停车")
                self._escape_wait_logged_at = now
            return
        if self._qr_escape_start_amcl is None:
            self._qr_escape_started_at = now
            if not self.prepare_qr_escape():
                return
        x, y, _ = self.amcl_pose
        moved = math.hypot(
            x - self._qr_escape_start_amcl[0], y - self._qr_escape_start_amcl[1]
        )
        selected_speed, selected_turn, reason = self.choose_escape_motion(
            self._escape_preferred_turn
        )
        if now - self._qr_escape_started_at >= self._qr_escape_timeout:
            self.stop_qr_escape()
            self.get_logger().warn(
                f"[脱困] 动作超时：实际位移={moved:.2f}m，重新规划"
            )
            self.resend_qr_route()
            return

        if selected_speed is None:
            self.escape_cmd_pub.publish(Twist())
            if self._escape_wait_logged_at is None or now - self._escape_wait_logged_at >= 1.0:
                self.get_logger().warn(f"[脱困] 前后脱困弧线都受阻，继续停车等待：{reason}")
                self._escape_wait_logged_at = now
            return

        self._escape_speed = selected_speed
        self._qr_escape_turn = selected_turn
        if moved < self._qr_escape_distance:
            cmd = Twist()
            cmd.linear.x = self._escape_speed
            cmd.angular.z = self._qr_escape_turn
            self.escape_cmd_pub.publish(cmd)
            return

        self.stop_qr_escape()
        self.get_logger().warn(f"[脱困] 动作结束：实际位移={moved:.2f}m，重新规划")
        self.resend_qr_route()

    def stop_qr_escape(self):
        cmd = Twist()
        self.escape_cmd_pub.publish(cmd)
        if self._qr_escape_timer is not None:
            self._qr_escape_timer.cancel()
            self._qr_escape_timer = None
        self._qr_escape_started_at = None
        self._qr_escape_start_amcl = None
        self._escape_speed = 0.0
        self._qr_escape_turn = 0.0
        self._escape_preferred_turn = 0.0
        self._escape_wait_logged_at = None

    def resend_qr_route(self):
        route_poses = self.route_poses()
        remaining = route_poses[self._qr_route_offset:]
        if not remaining:
            self.get_logger().error("[追点] 重发时没有剩余路点，保留最终目标逻辑")
            return
        self._last_visited = None
        self._qr_last_progress_time = None
        self._qr_last_amcl = None
        self._qr_last_remaining = None
        self._qr_replan_requested = False
        self._active_route_poses = remaining
        self._active_route_offset = self._qr_route_offset
        self.total_poses = len(remaining)
        self.get_logger().warn(
            f"[追点] 重新发送剩余路线：起始逻辑点={self._qr_route_offset + 1}，"
            f"剩余={len(remaining)}"
        )
        self.send_goal(remaining)

    def retry_first_phase_route(self):
        """全局图短暂误判起点时，保留 dating.csv 并请求新的规划。"""
        if self._first_phase_retry_timer is not None:
            self._first_phase_retry_timer.cancel()
            self._first_phase_retry_timer = None
        if self.first_phase_done or self.qr_result is not None:
            return
        self._last_visited = None
        self._active_route_poses = self._dating_route_poses
        self._active_route_offset = 0
        self.total_poses = len(self._dating_route_poses)
        self.get_logger().warn("[首阶段] 全局规划失败但局部未 lethal，重发 dating.csv 重新规划")
        self.send_goal(self._dating_route_poses)

    def goal_response_callback(self, future):
        self._goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"首阶段目标响应异常：{exc}")
            if self.qr_result is not None and not self.first_phase_done:
                self.first_phase_done = True
                self.execute_next_phase()
            return
        if not goal_handle.accepted:
            self.get_logger().error("路点任务被拒绝！")
            if self.qr_result is not None and not self.first_phase_done:
                self.first_phase_done = True
                self.execute_next_phase()
            return

        self._goal_handle = goal_handle # 保存当前句柄
        self.get_logger().info("路点任务已接受！")
        if self.qr_result is not None and not self.first_phase_done:
            self.get_logger().info("检测到二维码已提前到达，目标接受后立即取消首阶段任务")
            self.cancel_first_phase_goal()
        self.log_amcl("任务开始")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def cancel_response_callback(self, future):
        try:
            response = future.result()
            count = len(response.goals_canceling)
            self.get_logger().info(f"取消请求已返回：服务端接受 {count} 个目标")
        except Exception as exc:
            self.get_logger().error(f"取消请求失败：{exc}")

    def cancel_first_phase_goal(self):
        if self._goal_handle is None or self._first_phase_cancel_requested:
            return
        self._first_phase_cancel_requested = True
        self.get_logger().info("首阶段取消请求已发送")
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_response_callback)

    def get_result_callback(self, future):
        self._goal_handle = None
        self._goal_pending = False
        status = future.result().status
        status_names = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        status_name = status_names.get(status, f"UNKNOWN({status})")
        self.get_logger().info(f"导航任务结束，状态：{status_name}")

        if self._qr_replan_requested:
            self._goal_handle = None
            if self._qr_escape_replan_pending:
                self._qr_escape_replan_pending = False
                self.start_qr_escape()
            else:
                self.resend_qr_route()
            return

        if not self.first_phase_done:
            if status == GoalStatus.STATUS_CANCELED and self.qr_result is None:
                self.get_logger().error("第一阶段被取消，但未收到二维码，不能按正常完成处理")
                return
            if status == GoalStatus.STATUS_ABORTED:
                local_cost = (
                    self.local_footprint_cost(self._local_costmap)
                    if self._local_costmap is not None else None
                )
                if local_cost is None or local_cost < self.ESCAPE_COST:
                    self.get_logger().warn(
                        f"dating.csv 规划 ABORTED，但局部车体代价={local_cost}，"
                        f"{self.FIRST_PHASE_RETRY_DELAY:.1f}s 后重试"
                    )
                    self._first_phase_retry_timer = self.create_timer(
                        self.FIRST_PHASE_RETRY_DELAY, self.retry_first_phase_route
                    )
                else:
                    self.get_logger().error(
                        f"dating.csv 规划 ABORTED，局部车体已 lethal({local_cost})，保持停车"
                    )
                return
            self.first_phase_done = True
            if status == GoalStatus.STATUS_CANCELED and self.qr_result is not None:
                self.get_logger().warn("第一阶段因扫码被主动取消，开始执行二维码路线")
                self.first_phase_done = True
                self.execute_next_phase()
                return
            elif self.qr_result is None:
                self.get_logger().info("dating.csv 已完成，等待二维码识别...")
            else:
                self.execute_next_phase()
            return

        if status == GoalStatus.STATUS_ABORTED:
            route_poses = self.route_poses()
            current_index = self._qr_route_offset + (
                self._last_visited if self._last_visited is not None else 0
            )
            if current_index < len(route_poses) - 1:
                attempts = self._qr_escape_attempts.get(current_index, 0)
                self._qr_route_offset = current_index if attempts == 0 else current_index + 1
                self._qr_escape_attempts[current_index] = attempts + 1
                self.request_qr_escape(
                    f"Nav2 返回 ABORTED，目标[{current_index + 1}]"
                    + (" 首次失败，脱困后重试" if attempts == 0 else " 再次失败，脱困后跳过")
                )
                return

            if not self.first_phase_done:
                self.get_logger().error("第一阶段导航异常失败，已尝试脱困但没有可继续路点")
                return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"二维码路线未成功完成，状态：{status_name}")
            rclpy.shutdown()
            return
        self.get_logger().info("所有路点导航完成！")
        rclpy.shutdown()

    def execute_next_phase(self):
        if self.qr_result is None:
            self.get_logger().info("dating.csv 已完成，等待二维码...")
            return

        self.get_logger().info(f"使用二维码结果：{self.qr_result}")

        if self.qr_result % 2 == 1:
            self.get_logger().info("奇数：执行完整 shunshizhen1.csv，进入右上图生文区域后触发拍照")
            next_wp = self.read_waypoints_from_csv("/root/ros2_ws/src/racecar/scripts/point/shunshizhen1.csv")
        else:
            self.get_logger().info("偶数：执行 test_1.csv，进入左上图生文区域后触发拍照")
            next_wp = self.read_waypoints_from_csv("/root/ros2_ws/src/racecar/scripts/point/test_1.csv")

        next_poses = [self.make_pose(x, y, z, w) for x, y, z, w in next_wp]
        if self.qr_result % 2 == 1 and next_poses:
            final_pose = next_poses[-1]
            next_poses[-1] = self.make_pose(
                self.FINAL_RETURN_X,
                self.FINAL_RETURN_Y,
                final_pose.pose.orientation.z,
                final_pose.pose.orientation.w,
            )
            self.get_logger().info(
                f"奇数路线最后目标改为回起点："
                f"x={self.FINAL_RETURN_X:.3f} y={self.FINAL_RETURN_Y:.3f}（忽略航向）"
            )
        self.total_poses = len(next_poses)
        self._qr_route_poses = next_poses
        self._qr_route_offset = 0
        self._qr_replan_requested = False
        self._qr_last_progress_time = None
        self._qr_last_amcl = None
        self._qr_last_remaining = None
        self._qr_target_index = None
        self._qr_target_started_at = None
        self._qr_escape_attempts = {}
        self._active_route_poses = next_poses
        self._active_route_offset = 0
        self.photo_triggered = False
        self._last_visited = None
        self.get_logger().info(f"二维码路线整段导航启动：共 {self.total_poses} 个点")
        self.send_goal(next_poses)

    def make_pose(self, x, y, z, w):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = feedback.distance_remaining
        if self.total_poses > 0:
            visited = self.total_poses - feedback.number_of_poses_remaining
            if visited != self._last_visited:
                self.get_logger().info(
                    "剩余距离：{0:.2f} m | 已过 {1}/{2} 个点".format(remaining, visited, self.total_poses)
                )
                self.log_amcl(f"到达路点 {visited}")
                self._last_visited = visited
            self._qr_last_remaining = visited
        else:
            self.get_logger().info(
                "剩余距离：{0:.2f} m | 剩余路点 {1} 个".format(remaining, feedback.number_of_poses_remaining),
                once=True,
            )

        if self.first_phase_done and not self.photo_triggered and self.amcl_pose is not None:
            amcl_x, amcl_y, _ = self.amcl_pose
            in_photo_area = (
                (self.qr_result % 2 == 1 and amcl_x > 3.7 and amcl_y > 3.0)
                or (self.qr_result % 2 == 0 and amcl_x < 1.3 and amcl_y > 3.0)
            )
            if in_photo_area:
                msg = String()
                msg.data = "gaol"
                self.text_pub.publish(msg)
                self.photo_triggered = True

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

    client.get_logger().info("h1 已启动：先执行 dating.csv，扫码后切换对应路线")
    client.total_poses = len(poses)
    client._dating_route_poses = poses
    client._qr_escape_attempts = {}
    client._active_route_poses = poses
    client._active_route_offset = 0
    client.send_goal(poses)
    rclpy.spin(client)

if __name__ == '__main__':
    main()
