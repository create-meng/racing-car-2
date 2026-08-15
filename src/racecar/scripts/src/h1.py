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
    # 二维码路线当前目标的直接完成/删除半径；命中一次立即提交，不做连续帧确认。
    POINT_REACHED_RADIUS = 0.30
    # 253 只是膨胀层的 inscribed proximity，不能据此取消可通行路径；
    # 只有 254 才表示 footprint 实际进入 lethal 障碍区。
    ESCAPE_TRIGGER_COST = 254
    ESCAPE_BLOCK_COST = 254
    ESCAPE_CONFIRMATIONS = 3
    ESCAPE_SPEED = -0.05
    ESCAPE_DISTANCE = 0.15
    ESCAPE_REAR_CLEARANCE = 0.30
    ESCAPE_FRONT_CLEARANCE = 0.20
    FIRST_PHASE_RETRY_DELAY = 1.0
    DATING_WINDOW_SIZE = 1
    # NavigateThroughPoses 只保证最后一个目标成功，不保证中间目标到达；
    # 二维码路线必须逐点提交，否则会把未到达的点一起删掉。
    QR_WINDOW_SIZE = 1
    QR_PASS_CORRIDOR = 0.45

    def __init__(self):
        super().__init__('nav_through_poses_client')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._goal_handle = None # 用于记录当前任务句柄
        self._goal_pending = False  # 首阶段目标已发送，但服务端尚未返回句柄
        self._first_phase_cancel_requested = False
        self._qr_route_poses = []
        self._qr_route_offset = 0
        self._qr_window_size = self.QR_WINDOW_SIZE
        self._qr_replan_requested = False
        self._qr_last_progress_time = None
        self._qr_last_amcl = None
        self._qr_last_remaining = None
        self._qr_stall_timeout = 12.0
        self._qr_escape_distance = 0.22
        self._qr_escape_timeout = 5.0
        self._qr_escape_timer = None
        self._qr_escape_started_at = None
        self._qr_escape_start_amcl = None
        self._qr_escape_turn = 0.0
        self._escape_speed = 0.0
        self._escape_preferred_turn = 0.0
        self._escape_prefer_reverse = False
        self._qr_escape_replan_pending = False
        self._qr_point_completion_pending = False
        self._qr_escape_attempts = {}
        self._costmap_escape_hits = 0
        self._escape_requires_clear = False
        self._local_costmap = None
        self._latest_scan = None
        self._escape_wait_logged_at = None
        self._active_route_poses = []
        self._active_route_offset = 0
        self._dating_route_poses = []
        self._dating_route_offset = 0
        self._first_phase_retry_timer = None
        self._goal_retry_timer = None
        self._pending_goal_poses = None

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
        if cost is None or cost < self.ESCAPE_TRIGGER_COST:
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
        level = "lethal"
        front_clearance = self.scan_clearance(True)
        rear_clearance = self.scan_clearance(False)
        self.request_qr_escape(
            f"footprint 连续 {self._costmap_escape_hits} 帧进入 {level} 代价 {cost}，"
            f"前={front_clearance if front_clearance is not None else '--'}m，"
            f"后={rear_clearance if rear_clearance is not None else '--'}m",
            prefer_reverse=True,
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
        current_blocked = sum(cost >= self.ESCAPE_TRIGGER_COST for cost in current_costs)
        step_distance = 0.025
        for _ in range(int(self.ESCAPE_DISTANCE / step_distance)):
            dt = step_distance / abs(speed)
            x += speed * math.cos(yaw) * dt
            y += speed * math.sin(yaw) * dt
            yaw += turn * dt
            costs = self.footprint_costs_at(self._local_costmap, x, y, yaw)
            if not costs or max(costs) >= self.ESCAPE_BLOCK_COST:
                return False, f"{direction}弧线进入 lethal"
            if sum(cost >= self.ESCAPE_TRIGGER_COST for cost in costs) > current_blocked:
                return False, f"{direction}弧线更深入障碍"
        return True, ""

    def choose_escape_motion(self, preferred_turn):
        reverse = (
            (self.ESCAPE_SPEED, preferred_turn),
            (self.ESCAPE_SPEED, -preferred_turn),
        )
        forward = (
            (-self.ESCAPE_SPEED, preferred_turn),
            (-self.ESCAPE_SPEED, -preferred_turn),
        )
        alternatives = reverse + forward if self._escape_prefer_reverse else forward + reverse
        reasons = []
        for speed, turn in alternatives:
            clear, reason = self.escape_path_is_clear(speed, turn)
            if clear:
                return speed, turn, ""
            reasons.append(reason)
        return None, None, "；".join(reasons)

    def request_qr_escape(self, reason, prefer_reverse=False):
        """二维码路线的所有脱困触发统一取消任务，再进入同一脱困状态。"""
        if not self.first_phase_done or self.qr_result is None or self._qr_replan_requested:
            return
        self._qr_replan_requested = True
        self._qr_escape_replan_pending = True
        self._escape_prefer_reverse = prefer_reverse
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

        # Nav2 启动期间 action server 可能尚未 active，不能只等待一次后永久丢失路线。
        # 保存整段目标，由定时器在定位和 action server 就绪后重发。
        if self.amcl_pose is None:
            self._pending_goal_poses = list(safe_poses)
            self._start_goal_retry_timer("等待 AMCL 定位")
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = safe_poses

        if not self._action_client.wait_for_server(timeout_sec=0.5):
            self._pending_goal_poses = list(safe_poses)
            self._start_goal_retry_timer("等待 navigate_through_poses 服务")
            return
        self._pending_goal_poses = None
        if self._goal_retry_timer is not None:
            self._goal_retry_timer.cancel()
            self._goal_retry_timer = None
        self.get_logger().info("发送安全路点，共 {0} 个".format(len(safe_poses)))
        self._goal_pending = True
        self._first_phase_cancel_requested = False
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def _start_goal_retry_timer(self, reason):
        if self._goal_retry_timer is None:
            self.get_logger().warn(f"[启动门控] {reason}，路线暂存，Nav2 就绪后自动发送")
            self._goal_retry_timer = self.create_timer(1.0, self._retry_pending_goal)

    def _retry_pending_goal(self):
        if self._pending_goal_poses is None:
            if self._goal_retry_timer is not None:
                self._goal_retry_timer.cancel()
                self._goal_retry_timer = None
            return
        if self._goal_handle is not None or self._goal_pending:
            return
        self.send_goal(self._pending_goal_poses)

    def route_poses(self):
        return self._qr_route_poses if self.first_phase_done else self._dating_route_poses

    def monitor_route(self):
        """全程打印 AMCL 与当前目标；二维码路线仅在异常时重规划。"""
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

        # 当前二维码目标一旦进入完成半径，立即提交并从 Nav2 窗口删除；
        # 不再等待 number_of_poses_remaining，也不做连续帧确认。
        if distance <= self.POINT_REACHED_RADIUS:
            self._complete_qr_target_by_distance(target_index, distance)
            return

        now = self.get_clock().now().nanoseconds / 1e9
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
        if target_index < len(route_poses) - 1 and self._target_is_passed(target_index):
            self._qr_route_offset = target_index + 1
            reason = (
                f"{self._qr_stall_timeout:.0f}s 无位移但已确认经过目标[{target_index + 1}]，"
                f"脱困后从目标[{self._qr_route_offset + 1}]继续"
            )
        else:
            reason = (
                f"{self._qr_stall_timeout:.0f}s 无位移且未确认经过目标[{target_index + 1}]，"
                "保留当前目标并重规划"
            )
        self.request_qr_escape(reason)

    def _complete_qr_target_by_distance(self, target_index, distance):
        """按 AMCL 距离立即提交当前及其之前已确认的窗口点。"""
        if self._qr_replan_requested or self._qr_point_completion_pending:
            return

        local_completed = target_index - self._active_route_offset + 1
        if local_completed <= 0 or not self._active_route_poses:
            return
        local_completed = min(local_completed, len(self._active_route_poses))

        old_offset = self._qr_route_offset
        self._qr_route_offset = min(
            len(self._qr_route_poses), old_offset + local_completed
        )
        self._last_visited = None
        self._qr_point_completion_pending = True
        self._qr_replan_requested = True
        self._qr_escape_replan_pending = False
        self.get_logger().info(
            f"[点完成] 目标[{target_index + 1}] 距离={distance:.3f}m "
            f"<= {self.POINT_REACHED_RADIUS:.2f}m，立即删除并提交；"
            f"窗口提交 {local_completed} 个，路线索引 "
            f"{old_offset + 1} -> {self._qr_route_offset + 1}"
        )

        if self._goal_handle is None or self._goal_pending:
            self._qr_point_completion_pending = False
            if self._qr_route_offset >= len(self._qr_route_poses):
                self.get_logger().info("所有路点导航完成！")
                rclpy.shutdown()
            else:
                self.resend_qr_route()
            return

        self._goal_handle.cancel_goal_async().add_done_callback(
            self.cancel_response_callback
        )

    def _target_is_passed(self, target_index):
        """确认车辆沿当前路线方向经过目标，而不是仅仅离目标变远。"""
        route = self._qr_route_poses
        if self.amcl_pose is None or target_index <= 0 or target_index >= len(route):
            return False
        previous = route[target_index - 1].pose.position
        target = route[target_index].pose.position
        px, py, _ = self.amcl_pose
        dx = target.x - previous.x
        dy = target.y - previous.y
        length_sq = dx * dx + dy * dy
        if length_sq < 1.0e-6:
            return False
        projection = ((px - previous.x) * dx + (py - previous.y) * dy) / length_sq
        lateral = abs((px - previous.x) * dy - (py - previous.y) * dx) / math.sqrt(length_sq)
        return projection >= 1.0 and lateral <= self.QR_PASS_CORRIDOR

    def _commit_window_progress(self, reason):
        """只按 AMCL 距离提交连续完成的窗口点，不信任 Nav2 的 visited 计数。"""
        if not self.first_phase_done or not self._active_route_poses:
            return 0
        completed = self._count_reached_active_targets()
        if completed:
            old_offset = self._qr_route_offset
            self._qr_route_offset = min(
                len(self._qr_route_poses), self._qr_route_offset + completed
            )
            self.get_logger().info(
                f"[进度提交] {reason}：窗口完成 {completed} 个点，"
                f"路线索引 {old_offset + 1} -> {self._qr_route_offset + 1}"
            )
        self._last_visited = None
        return completed

    def _count_reached_active_targets(self):
        """返回从当前窗口首点开始、连续进入完成半径的点数。"""
        if self.amcl_pose is None:
            return 0
        amcl_x, amcl_y, _ = self.amcl_pose
        completed = 0
        for pose in self._active_route_poses:
            target = pose.pose.position
            if math.hypot(target.x - amcl_x, target.y - amcl_y) > self.POINT_REACHED_RADIUS:
                break
            completed += 1
        return completed

    def _commit_successful_window(self):
        """Nav2 成功后只提交 AMCL 已实际进入完成半径的连续点。"""
        count = self._count_reached_active_targets()
        if count == 0:
            self.get_logger().warn(
                "Nav2 返回 SUCCEEDED，但当前二维码目标未进入 "
                f"{self.POINT_REACHED_RADIUS:.2f}m；保留当前点并重发"
            )
            self._last_visited = None
            return
        old_offset = self._qr_route_offset
        self._qr_route_offset = min(len(self._qr_route_poses), old_offset + count)
        self.get_logger().info(
            f"[进度提交] 窗口成功：{count} 个点，"
            f"路线索引 {old_offset + 1} -> {self._qr_route_offset + 1}"
        )
        self._last_visited = None

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
            # 后方空间足够时多退一点；空间紧时保留最小脱困距离，不能越过后方障碍。
            if selected_speed < 0.0:
                rear_clearance = self.scan_clearance(False)
                if rear_clearance is not None:
                    self._qr_escape_distance = min(
                        0.35, max(0.22, rear_clearance - 0.10)
                    )
                else:
                    self._qr_escape_distance = 0.22
            else:
                self._qr_escape_distance = 0.22
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
        self._escape_prefer_reverse = False
        self._qr_escape_distance = 0.22
        self._escape_wait_logged_at = None

    def resend_qr_route(self):
        self._commit_window_progress("重新规划前")
        route_poses = self.route_poses()
        remaining = route_poses[self._qr_route_offset:]
        if not remaining:
            self.get_logger().error("[追点] 重发时没有剩余路点，保留最终目标逻辑")
            return
        window = remaining[:self._qr_window_size]
        self._last_visited = None
        self._qr_last_progress_time = None
        self._qr_last_amcl = None
        self._qr_last_remaining = None
        self._escape_prefer_reverse = False
        self._qr_replan_requested = False
        self._active_route_poses = window
        self._active_route_offset = self._qr_route_offset
        self.total_poses = len(window)
        self.get_logger().warn(
            f"[追点] 重新发送剩余路线：起始逻辑点={self._qr_route_offset + 1}，"
            f"本窗口={len(window)}，全路线剩余={len(remaining)}"
        )
        self.send_goal(window)

    def retry_first_phase_route(self):
        """只重发首阶段当前窗口，避免后续点阻塞起步。"""
        if self._first_phase_retry_timer is not None:
            self._first_phase_retry_timer.cancel()
            self._first_phase_retry_timer = None
        if self.first_phase_done or self.qr_result is not None:
            return
        self.get_logger().warn("[首阶段] 当前窗口规划失败但局部未 lethal，重发当前 dating 点")
        self.resend_dating_route()

    def resend_dating_route(self):
        """首阶段按单点窗口规划，后续点不能阻塞当前点起步。"""
        remaining = self._dating_route_poses[self._dating_route_offset:]
        if not remaining:
            self.get_logger().error("[首阶段] 重发时没有剩余 dating 路点")
            return
        window = remaining[:self.DATING_WINDOW_SIZE]
        self._last_visited = None
        self._active_route_poses = window
        self._active_route_offset = self._dating_route_offset
        self.total_poses = len(window)
        self.get_logger().info(
            f"[首阶段] 发送窗口：逻辑点 {self._dating_route_offset + 1}/"
            f"{len(self._dating_route_poses)}，本窗口 {len(window)} 个点"
        )
        self.send_goal(window)

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
            elif self._qr_route_offset >= len(self._qr_route_poses):
                self._qr_point_completion_pending = False
                self.get_logger().info("所有路点导航完成！")
                rclpy.shutdown()
            else:
                self._qr_point_completion_pending = False
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
                if local_cost is None or local_cost < self.ESCAPE_BLOCK_COST:
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
            if status == GoalStatus.STATUS_CANCELED and self.qr_result is not None:
                self.get_logger().warn("第一阶段因扫码被主动取消，开始执行二维码路线")
                self.first_phase_done = True
                self.execute_next_phase()
                return
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(f"dating.csv 窗口未成功完成，状态：{status_name}")
                return
            self._dating_route_offset += len(self._active_route_poses)
            if self._dating_route_offset < len(self._dating_route_poses):
                self.resend_dating_route()
                return
            self.first_phase_done = True
            if self.qr_result is None:
                self.get_logger().info("dating.csv 已完成，等待二维码识别...")
            else:
                self.execute_next_phase()
            return

        if status == GoalStatus.STATUS_ABORTED:
            self._commit_window_progress("Nav2 ABORTED 后")
            route_poses = self.route_poses()
            current_index = self._qr_route_offset
            if current_index < len(route_poses):
                attempts = self._qr_escape_attempts.get(current_index, 0)
                if current_index < len(route_poses) - 1 and attempts > 0 \
                        and self._target_is_passed(current_index):
                    self._qr_route_offset = current_index + 1
                self._qr_escape_attempts[current_index] = attempts + 1
                if current_index == len(route_poses) - 1:
                    reason = f"Nav2 返回 ABORTED，最终目标[{current_index + 1}]，脱困后重试"
                else:
                    reason = (
                        f"Nav2 返回 ABORTED，目标[{current_index + 1}]"
                        + (" 首次失败，脱困后重试" if attempts == 0
                           else " 再次失败，脱困后继续当前目标或按经过条件推进")
                    )
                self.request_qr_escape(reason)
                return

            if not self.first_phase_done:
                self.get_logger().error("第一阶段导航异常失败，已尝试脱困但没有可继续路点")
                return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"二维码路线未成功完成，状态：{status_name}")
            rclpy.shutdown()
            return
        self._commit_successful_window()
        if self._qr_route_offset < len(self._qr_route_poses):
            self._qr_replan_requested = False
            self.resend_qr_route()
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
        self._qr_escape_attempts = {}
        self._active_route_poses = next_poses
        self._active_route_offset = 0
        self.photo_triggered = False
        self._last_visited = None
        self.get_logger().info(
            f"二维码路线窗口导航启动：全路线 {self.total_poses} 个点，"
            f"首窗口最多 {self._qr_window_size} 个点"
        )
        self.resend_qr_route()

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
                global_index = self._active_route_offset + visited
                self.log_amcl(f"到达路线点 {global_index}")
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
    client._dating_route_offset = 0
    client._qr_escape_attempts = {}
    client.resend_dating_route()
    rclpy.spin(client)

if __name__ == '__main__':
    main()
