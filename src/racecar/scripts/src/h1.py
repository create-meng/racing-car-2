#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String  # 必须是String！
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from action_msgs.msg import GoalStatus
from nav2_msgs.srv import ClearEntireCostmap
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
import csv
import os
import re
import math
from collections import deque


class NavThroughPosesClient(Node):
    FINAL_RETURN_X = 0.5
    # y=0.2 时完成半径 0.30m 会让车停在 y=0.3~0.4 附近就判完成，
    # 实际没回到终点线；改 0.0 让完成判定以 (0.5, 0.0) 为中心。
    FINAL_RETURN_Y = 0.0
    # 二维码路线当前目标的直接完成/删除半径；命中一次立即提交，不做连续帧确认。
    POINT_REACHED_RADIUS = 0.30
    # 253 只是膨胀层的 inscribed proximity，不能据此取消可通行路径；
    # 只有 254 才表示 footprint 实际进入 lethal 障碍区。
    ESCAPE_TRIGGER_COST = 254
    ESCAPE_BLOCK_COST = 254
    ESCAPE_CONFIRMATIONS = 3
    ESCAPE_SPEED = -0.15
    ESCAPE_DISTANCE = 0.15
    ESCAPE_REAR_CLEARANCE = 0.25
    ESCAPE_FRONT_CLEARANCE = 0.15
    FIRST_PHASE_RETRY_DELAY = 0.2
    DATING_WINDOW_SIZE = 1
    # NavigateThroughPoses 只保证最后一个目标成功，不保证中间目标到达；
    # 二维码路线必须逐点提交，否则会把未到达的点一起删掉。
    QR_WINDOW_SIZE = 1
    QR_PASS_CORRIDOR = 0.45
    # Nav2 备用规划器异常成功时，最多允许一次重试；连续无进展立即进入统一脱困。
    QR_NO_PROGRESS_SUCCESS_LIMIT = 2
    # 卡死检测（AMCL 位置滚动窗口净位移）：原地打轮/蠕动/完全停车都算卡死。
    # 注意不能用 odom：顶住障碍时轮子空转会让 odom 报"在动"，只有 AMCL 位置
    # 反映真实物理位移；也不能按 AMCL"有无发布"判断——打轮时 yaw 抖动会
    # 触发发布，把"无位移"误判成"有进展"，导致卡 10s+ 才脱困。
    STALL_NET_DISP = 0.08       # 窗口内位置净位移 < 8cm = 无真实进展
    STALL_BLOCKED_FRONT = 0.40  # 前方障碍 < 0.40m 且无进展 → 快速脱困
    # 脱困转向物理上限：Ackermann 最小转弯半径 0.35m（与 MPPI 运动模型一致）。
    # 弧线检查按命令转向模拟轨迹，若命令超出物理能力，模拟弧线比真实更急，
    # 会把"真实可走的弧线"误判为撞障碍 → 频繁"停车等待"。
    ESCAPE_MIN_RADIUS = 0.35
    # 弧线全被挡时的提前结束策略：已拉开 ≥0.30m 且全挡 ≥1s，或原地全挡 ≥3s，
    # 直接结束脱困并重规划（实测此时重规划都能成功，原地等超时纯浪费时间）。
    ESCAPE_FINISH_MOVED = 0.30
    ESCAPE_FINISH_BLOCKED_S = 1.0
    ESCAPE_FINISH_BOXED_S = 3.0

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
        # 卡死判定：AMCL 位置滚动窗口净位移 < STALL_NET_DISP 持续 stall 秒即卡死。
        # 空旷卡死 3.5s；前方有障 / 已脱困过的目标 2.0s（果断处置，不等 MPPI 打轮）
        self._qr_stall_timeout = 3.5
        self._qr_stall_timeout_blocked = 2.0
        self._qr_escape_distance = 0.50
        # 10s：脱困含“位移拉开 + 车头对准目标”两阶段，摆头需要更多时间
        self._qr_escape_timeout = 10.0
        self._stall_window = deque()      # (t, amcl_x, amcl_y) 卡死检测滚动窗口
        self._escape_dir_locked = None    # 脱困方向锁定：-1 倒车 / +1 前进
        self._escape_all_blocked_since = None  # 弧线全挡起始时刻（提前结束用）
        self._qr_escape_timer = None
        self._qr_escape_started_at = None
        self._qr_escape_start_amcl = None
        self._qr_escape_start_odom = None
        self._qr_escape_turn = 0.0
        self._escape_speed = 0.0
        self._escape_preferred_turn = 0.0
        self._escape_prefer_reverse = False
        self._qr_escape_replan_pending = False
        self._qr_point_completion_pending = False
        self._qr_escape_attempts = {}
        self._qr_no_progress_successes = 0
        self._qr_final_goal_completed = False
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
        self._first_phase_failures = 0
        self._pullback_ticks = 0
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
        # odom 用于脱困位移判定：AMCL 在 update_min_d=0.1~0.2 下低速移动
        # 时不发布新位姿，用 AMCL 判位移会把“实际在挪”误判为 0。
        self.odom_pose = None
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom_combined",
            self.odom_callback,
            10,
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

        # 脱困结束后清理全局/局部代价地图：激光经 AMCL map->odom 变换后，
        # 全局图会残留“压在车体上的 254/253”脏障碍（13:23:43 全局254/局部0），
        # 不清图的话脱困后重规划会再次失败并继续撞障碍。
        # 注意：Nav2 清图服务类型是 nav2_msgs/srv/ClearEntireCostmap，
        # 用 std_srvs/Empty 会导致 service_is_ready 永远为 False（静默失败）。
        self._clear_global_client = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"
        )
        self._clear_local_client = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
        )

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

    def odom_callback(self, msg):
        """记录里程计位置 (odom_combined 系)，用于脱困位移判定。"""
        p = msg.pose.pose
        self.odom_pose = (p.position.x, p.position.y)

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
        """检查脱困弧线；允许从当前 lethal 栅格逐步驶出，但禁止更深入。"""
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
        # 车辆已经被代价地图判为 lethal 时，前几厘米仍可能落在同一栅格。
        # 不能要求每个中间采样点立刻变成非 lethal，否则脱困永远不会开始。
        # 只允许 lethal 数量保持或减少；一旦增加，说明运动方向在深入障碍。
        if current_blocked == 0:
            allow_existing_lethal = False
        else:
            allow_existing_lethal = True
        for _ in range(int(self.ESCAPE_DISTANCE / step_distance)):
            dt = step_distance / abs(speed)
            x += speed * math.cos(yaw) * dt
            y += speed * math.sin(yaw) * dt
            yaw += turn * dt
            costs = self.footprint_costs_at(self._local_costmap, x, y, yaw)
            if not costs:
                return False, f"{direction}弧线超出 costmap"
            blocked = sum(cost >= self.ESCAPE_TRIGGER_COST for cost in costs)
            if blocked > current_blocked:
                return False, f"{direction}弧线更深入障碍"
            if max(costs) >= self.ESCAPE_BLOCK_COST and not allow_existing_lethal:
                return False, f"{direction}弧线进入 lethal"
        if allow_existing_lethal:
            return True, f"{direction}允许从当前 lethal 逐步脱出"
        return True, ""

    def choose_escape_motion(self, preferred_turn):
        """按比例转向优先选择安全脱困弧线；方向首帧锁定，全程不翻转。

        转向阶梯：首选比例转向，被障碍挡时依次降转（0.6x/0.35x/直行），
        尽量给出可执行动作而不是停车等待——停车等待 = 原地打轮时间。
        方向锁定：一旦选定倒车/前进，后续帧只在该方向内选，避免
        “倒一点→前面通了→又往前冲→再被挡”的来回犹豫。
        """
        ladders = []
        for factor in (1.0, 0.6, 0.35, 0.0):
            turn = max(-1.0, min(1.0, preferred_turn * factor))
            if turn not in ladders:
                ladders.append(turn)
        if self._escape_dir_locked is not None:
            pairs = [(self._escape_dir_locked, t) for t in ladders]
        elif self._escape_prefer_reverse:
            pairs = [(-1, t) for t in ladders] + [(1, t) for t in ladders]
        else:
            pairs = [(1, t) for t in ladders] + [(-1, t) for t in ladders]
        reasons = []
        for sign, turn in pairs:
            speed = self.ESCAPE_SPEED if sign < 0 else -self.ESCAPE_SPEED
            clear, reason = self.escape_path_is_clear(speed, turn)
            if clear:
                if self._escape_dir_locked is None:
                    self._escape_dir_locked = sign
                return speed, turn, reason
            reasons.append(reason)
        return None, None, "；".join(reasons)

    def request_qr_escape(self, reason, prefer_reverse=False):
        """二维码路线的所有脱困触发统一取消任务，再进入同一脱困状态。"""
        if not self.first_phase_done or self.qr_result is None or self._qr_replan_requested:
            return

        # 连续多次脱困仍无法推进时，若目标方向持续被障碍占据则跳过该点继续路线，
        # 保证“整条路线完成优先级最高，不能永久卡在某一点”。
        route_poses = self.route_poses()
        if self._active_route_poses and route_poses:
            idx = self._qr_route_offset
            attempts = self._qr_escape_attempts.get(idx, 0)
            if attempts >= 4 and idx < len(route_poses):
                target = self._active_route_poses[0].pose.position
                if self.amcl_pose is not None:
                    distance = math.hypot(
                        target.x - self.amcl_pose[0], target.y - self.amcl_pose[1]
                    )
                    front = self.scan_clearance(True)
                    if distance < 0.8 and front is not None and front < 0.6:
                        self.get_logger().error(
                            f"[跳过障碍点] 目标[{idx + 1}] 已脱困 {attempts} 次仍被障碍占据"
                            f"（距离={distance:.2f}m，前方={front:.2f}m），跳过该点继续路线"
                        )
                        self._qr_escape_attempts.pop(idx, None)
                        self._qr_route_offset = min(len(self._qr_route_poses), idx + 1)
                        self._qr_replan_requested = True
                        self._qr_escape_replan_pending = False
                        if self._goal_handle is not None and not self._goal_pending:
                            self._goal_handle.cancel_goal_async().add_done_callback(
                                self.cancel_response_callback
                            )
                        else:
                            self.resend_qr_route()
                        return

        self._qr_no_progress_successes = 0
        self._qr_replan_requested = True
        self._qr_escape_replan_pending = True
        self._escape_prefer_reverse = prefer_reverse
        # 统一在此累计当前目标的脱困次数（ABORTED/无位移/假成功都走这里）
        self._qr_escape_attempts[self._qr_route_offset] = (
            self._qr_escape_attempts.get(self._qr_route_offset, 0) + 1
        )
        self.get_logger().warn(f"[脱困] {reason}，取消当前导航并选择安全动作")
        # 立即发零速：取消握手期间 MPPI 仍在原地打轮，先接管命令停车
        self.escape_cmd_pub.publish(Twist())
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
        # 卡死检测（dating 与二维码共用）：AMCL 位置滚动窗口净位移。
        # AMCL 的 update_min_d/a=0.1 只在 odom 移动超阈值时发布新位姿——
        # 原地打轮时 yaw 抖动会触发发布，但位置几乎不变。旧的"按 AMCL
        # 有无移动刷新进度"逻辑会被打轮不断续命，卡 10s+ 才脱困；
        # 这里只看窗口内【位置】净位移：< STALL_NET_DISP 且持续够久 = 卡死。
        # 也不用 odom：顶住障碍空转时 odom 会报"在动"，只有 AMCL 位置真实。
        now = self.get_clock().now().nanoseconds / 1e9
        stuck = False
        blocked_ahead = False
        if not self._qr_replan_requested:
            attempts = self._qr_escape_attempts.get(target_index, 0)
            front = self.scan_clearance(True)
            blocked_ahead = front is not None and front < self.STALL_BLOCKED_FRONT
            stall = (
                self._qr_stall_timeout_blocked
                if (blocked_ahead or attempts > 0)
                else self._qr_stall_timeout
            )
            self._stall_window.append((now, amcl_x, amcl_y))
            while len(self._stall_window) > 1 and now - self._stall_window[0][0] > 6.0:
                self._stall_window.popleft()
            if len(self._stall_window) > 1:
                t0, x0, y0 = self._stall_window[0]
                net = math.hypot(amcl_x - x0, amcl_y - y0)
                if net >= self.STALL_NET_DISP:
                    # 有真实位移：清窗口，从当前位置重新计时
                    self._stall_window.clear()
                    self._stall_window.append((now, amcl_x, amcl_y))
                elif now - t0 >= stall:
                    stuck = True

        if not stuck:
            # 二维码目标进入完成半径 → 立即提交；dating 由结果回调推进
            if not self.first_phase_done:
                return
            if distance <= self.POINT_REACHED_RADIUS:
                self._complete_qr_target_by_distance(target_index, distance)
            return

        if not self.first_phase_done:
            self.handle_dating_stall()
            return

        route_poses = self.route_poses()
        if target_index < len(route_poses) - 1 and self._target_is_passed(target_index):
            self._qr_route_offset = target_index + 1
            reason = (
                f"{stall:.1f}s 无净位移但已确认经过目标[{target_index + 1}]，"
                f"脱困后从目标[{self._qr_route_offset + 1}]继续"
            )
        else:
            reason = (
                f"{stall:.1f}s 无净位移（<{self.STALL_NET_DISP:.2f}m）"
                f"且未确认经过目标[{target_index + 1}]，保留当前目标并重规划"
            )
        self.request_qr_escape(reason, prefer_reverse=blocked_ahead)

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
        if self._qr_route_offset >= len(self._qr_route_poses):
            self._qr_final_goal_completed = True
        self._last_visited = None
        self._qr_no_progress_successes = 0
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
                if not self._qr_final_goal_completed:
                    self._qr_route_offset = max(0, len(self._qr_route_poses) - 1)
                    self.get_logger().error(
                        "[最终回点保护] 索引到达末尾但最终目标未确认完成，重新发送最终目标"
                    )
                    self.resend_qr_route()
                else:
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
            if self._qr_route_offset >= len(self._qr_route_poses):
                self._qr_final_goal_completed = True
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
            return 0
        old_offset = self._qr_route_offset
        self._qr_route_offset = min(len(self._qr_route_poses), old_offset + count)
        if self._qr_route_offset >= len(self._qr_route_poses):
            self._qr_final_goal_completed = True
        self.get_logger().info(
            f"[进度提交] 窗口成功：{count} 个点，"
            f"路线索引 {old_offset + 1} -> {self._qr_route_offset + 1}"
        )
        self._last_visited = None
        self._qr_no_progress_successes = 0
        return count

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
        # 转向上限按物理转弯半径限制：turn_max = v / r_min。
        # 命令超物理的转向会让弧线检查模拟出真实做不出的急弧，误判撞障碍。
        turn_cap = abs(self.ESCAPE_SPEED) / self.ESCAPE_MIN_RADIUS
        self._qr_escape_turn = max(-turn_cap, min(turn_cap, 1.2 * angle_error))
        # 不强制最小转向：小误差时转向自然小，避免过零翻转/转过头
        self._escape_preferred_turn = self._qr_escape_turn
        # 本次脱困方向未锁定：由首个安全动作决定（倒车/前进全程一致）
        self._escape_dir_locked = None
        self._escape_all_blocked_since = None
        self._qr_escape_start_amcl = (x, y)
        self._qr_escape_start_odom = self.odom_pose
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
            if reason:
                self.get_logger().info(f"[脱困] 轨迹检查通过：{reason}")
            # 后退距离 = 前方障碍距离 + 0.20m 净空（让车头完全脱离障碍膨胀区），
            # 限幅 0.40~0.50m；再以"后方净空-0.15"封顶，避免退进后方锥桶。
            # 之前按后方净空算（0.40~0.65m）会退过头：既浪费时间，
            # 又常把车退进后方锥桶堆里导致弧线全被挡、原地停车。
            if selected_speed < 0.0:
                front_clearance = self.scan_clearance(True)
                rear_clearance = self.scan_clearance(False)
                if front_clearance is not None:
                    self._qr_escape_distance = max(
                        0.40, min(0.50, front_clearance + 0.20)
                    )
                else:
                    self._qr_escape_distance = 0.50
                if rear_clearance is not None:
                    self._qr_escape_distance = min(
                        self._qr_escape_distance, max(0.35, rear_clearance - 0.15)
                    )
            else:
                self._qr_escape_distance = 0.50
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
        # 脱困位移优先用 odom 判定：AMCL 在 update_min_d 阈值内不发布，
        # 低速脱困时会把“实际在挪”误判为 0.00m；odom 始终连续。
        moved = None
        if self._qr_escape_start_odom is not None and self.odom_pose is not None:
            moved = math.hypot(
                self.odom_pose[0] - self._qr_escape_start_odom[0],
                self.odom_pose[1] - self._qr_escape_start_odom[1],
            )
        if moved is None and self._qr_escape_start_amcl is not None:
            x, y, _ = self.amcl_pose
            moved = math.hypot(
                x - self._qr_escape_start_amcl[0], y - self._qr_escape_start_amcl[1]
            )
        if moved is None:
            moved = 0.0

        # 转向用“比例控制 + 死区”：turn = clamp(1.2*yaw_error, ±v/r_min)，
        # 误差小时自然降转、±0.2rad 内不转 → 车头连续收敛，不振荡（每帧全速翻转）
        # 也不过头（方向锁定导致转过目标 100°+，脱困完车头反了）。
        # 转向上限按当前速度的物理转弯半径限制：超物理的转向会让弧线检查
        # 模拟出真实做不出的急弧，把可走弧线误判成撞障碍。
        # 位移已达标进入摆头阶段时，按摆头速度 0.05m/s 的物理上限选转。
        x, y, yaw = self.amcl_pose
        local_target = min(
            self._last_visited if self._last_visited is not None else 0,
            len(self._active_route_poses) - 1,
        )
        target = self._active_route_poses[local_target].pose.position
        target_heading = math.atan2(target.y - y, target.x - x)
        yaw_error = math.atan2(
            math.sin(target_heading - yaw), math.cos(target_heading - yaw)
        )
        swing_mode = moved >= self._qr_escape_distance
        speed_for_cap = 0.05 if swing_mode else (
            abs(self._escape_speed) or abs(self.ESCAPE_SPEED)
        )
        turn_cap = speed_for_cap / self.ESCAPE_MIN_RADIUS
        preferred_turn = max(-turn_cap, min(turn_cap, 1.2 * yaw_error))
        if abs(yaw_error) < 0.2:
            preferred_turn = 0.0  # 死区：基本对准时直行/直退拉开，不再转向

        selected_speed, selected_turn, reason = self.choose_escape_motion(
            preferred_turn
        )
        if now - self._qr_escape_started_at >= self._qr_escape_timeout:
            self.stop_qr_escape()
            self.get_logger().warn(
                f"[脱困] 动作超时：实际位移={moved:.2f}m，重新规划"
            )
            self.resend_qr_route()
            return

        if selected_speed is None:
            # 所有弧线都被挡：先短暂观察；持续被挡则提前结束脱困并重规划，
            # 不再原地等到 10s 超时（实测结束+重规划都能立即成功）。
            if self._escape_all_blocked_since is None:
                self._escape_all_blocked_since = now
            finish = (
                now - self._escape_all_blocked_since >= self.ESCAPE_FINISH_BLOCKED_S
                and moved >= self.ESCAPE_FINISH_MOVED
            ) or now - self._escape_all_blocked_since >= self.ESCAPE_FINISH_BOXED_S
            if finish:
                blocked_s = now - self._escape_all_blocked_since
                self.stop_qr_escape()
                self.get_logger().warn(
                    f"[脱困] 已拉开 {moved:.2f}m 但弧线持续受阻 "
                    f"{blocked_s:.1f}s，提前结束并重规划"
                )
                self.resend_qr_route()
                return
            self.escape_cmd_pub.publish(Twist())
            if self._escape_wait_logged_at is None or now - self._escape_wait_logged_at >= 1.0:
                self.get_logger().warn(f"[脱困] 前后脱困弧线都受阻，继续停车等待：{reason}")
                self._escape_wait_logged_at = now
            return

        self._escape_speed = selected_speed
        self._qr_escape_turn = selected_turn
        self._escape_all_blocked_since = None

        # 脱困完成条件：位移拉开 且 车头对准目标方向（±20°）
        if moved >= self._qr_escape_distance and abs(yaw_error) <= 0.35:
            self.stop_qr_escape()
            self.get_logger().warn(
                f"[脱困] 动作结束：位移={moved:.2f}m，朝向误差="
                f"{math.degrees(yaw_error):.0f}°，重新规划"
            )
            self.resend_qr_route()
            return

        cmd = Twist()
        cmd.linear.x = self._escape_speed
        cmd.angular.z = self._qr_escape_turn
        # 位移已达标但朝向未对准：放慢到 0.05m/s，保持大转向继续摆头。
        # 保持 choose_escape_motion 选定的方向（正车/倒车），只降速；
        # 摆头转向同样按低速的物理转弯半径限幅（否则弧线检查又会误判）。
        if moved >= self._qr_escape_distance:
            cmd.linear.x = 0.05 if self._escape_speed > 0.0 else -0.05
            swing_cap = 0.05 / self.ESCAPE_MIN_RADIUS
            cmd.angular.z = max(-swing_cap, min(swing_cap, cmd.angular.z))
        self.escape_cmd_pub.publish(cmd)

    def clear_costmaps(self):
        """清空全局/局部代价地图的障碍层（异步，不阻塞）。

        脱困结束后调用：激光经 AMCL map->odom 变换会在全局图留下
        “压在车体上的 254/253”脏障碍，不清掉则重规划仍会失败。
        """
        for client in (self._clear_global_client, self._clear_local_client):
            if client.service_is_ready():
                future = client.call_async(ClearEntireCostmap.Request())
                future.add_done_callback(
                    lambda f, name=client.srv_name: (
                        self.get_logger().info(f"[清图] 成功 {name}")
                        if f.result() is not None
                        else self.get_logger().warn(f"[清图] 失败 {name}")
                    )
                )
            else:
                self.get_logger().warn(f"[清图] 服务未就绪 {client.srv_name}")

    def stop_qr_escape(self):
        cmd = Twist()
        self.escape_cmd_pub.publish(cmd)
        if self._qr_escape_timer is not None:
            self._qr_escape_timer.cancel()
            self._qr_escape_timer = None
        self._qr_escape_started_at = None
        self._qr_escape_start_amcl = None
        self._qr_escape_start_odom = None
        self._escape_speed = 0.0
        self._qr_escape_turn = 0.0
        self._escape_preferred_turn = 0.0
        self._escape_prefer_reverse = False
        self._escape_dir_locked = None
        self._escape_all_blocked_since = None
        self._qr_escape_distance = 0.50
        self._escape_wait_logged_at = None
        # 脱困动作结束：清掉全局/局部障碍残影，让重规划基于干净地图。
        self.clear_costmaps()

    def resend_qr_route(self):
        # 进度只由结果/脱困处理器提交一次；重发函数不能再次提交旧窗口。
        route_poses = self.route_poses()
        remaining = route_poses[self._qr_route_offset:]
        if not remaining:
            if self._qr_final_goal_completed:
                self.get_logger().info("[追点] 最终回点已确认完成，无剩余路点")
                return
            if route_poses:
                self._qr_route_offset = len(route_poses) - 1
                remaining = route_poses[self._qr_route_offset:]
                self.get_logger().error(
                    "[最终回点保护] 重发前发现末尾索引异常，回退到最后一个目标"
                )
            else:
                self.get_logger().error("[追点] 路线为空，无法发送最终目标")
                return
        window = remaining[:self._qr_window_size]
        self._last_visited = None
        self._qr_last_progress_time = None
        self._qr_last_amcl = None
        self._qr_last_remaining = None
        self._stall_window.clear()
        self._escape_dir_locked = None
        self._escape_prefer_reverse = False
        self._qr_replan_requested = False
        self._active_route_poses = window
        self._active_route_offset = self._qr_route_offset
        self.total_poses = len(window)
        if self._active_route_offset + len(window) >= len(route_poses):
            final_target = window[-1].pose.position
            self.get_logger().warn(
                f"[最终回点] 已发送目标 x={final_target.x:.3f} y={final_target.y:.3f}"
            )
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

    def handle_dating_stall(self):
        """首阶段（dating.csv）卡死处置：清图快速重试；≥2 次则物理倒车拉开。

        与 ABORTED 分支共用失败计数，卡死不再死等 MPPI 打轮。
        """
        self._first_phase_failures += 1
        rear = self.scan_clearance(False)
        if self._first_phase_failures >= 2 and rear is not None and rear >= 0.30:
            self.get_logger().warn(
                f"[起步拉开] 首阶段卡死 {self._first_phase_failures} 次，"
                f"后方 {rear:.2f}m，倒车 0.22m 拉开后重试"
            )
            self.clear_costmaps()
            self._pullback_ticks = 0
            self._first_phase_retry_timer = self.create_timer(
                0.2, self._start_pullback_tick
            )
            return
        self.clear_costmaps()
        self.get_logger().warn(
            f"[首阶段] 卡死 {self._first_phase_failures} 次，清图后 "
            f"{self.FIRST_PHASE_RETRY_DELAY:.1f}s 快速重试"
        )
        self._first_phase_retry_timer = self.create_timer(
            self.FIRST_PHASE_RETRY_DELAY, self.retry_first_phase_route
        )

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

    def _start_pullback_tick(self):
        """起步拉开：每 0.2s 发一次倒车命令，共 1.8s（≈0.22m），然后重发首点。"""
        self._pullback_ticks += 1
        cmd = Twist()
        if self._pullback_ticks >= 9:
            self.escape_cmd_pub.publish(Twist())
            if self._first_phase_retry_timer is not None:
                self._first_phase_retry_timer.cancel()
                self._first_phase_retry_timer = None
            self.get_logger().warn("[起步拉开] 倒车结束，清图并重发 dating 首点")
            self.clear_costmaps()
            self.resend_dating_route()
            return
        cmd.linear.x = -0.12
        self.escape_cmd_pub.publish(cmd)

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
                if not self._qr_final_goal_completed:
                    self._qr_route_offset = max(0, len(self._qr_route_poses) - 1)
                    self.get_logger().error(
                        "[最终回点保护] 重规划回调到达末尾但最终目标未完成，重新发送最终目标"
                    )
                    self.resend_qr_route()
                else:
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
                    # 起步贴墙时 planner 间歇性失败（no valid path found）：
                    # 障碍单元与车体 footprint 在栅格离散下间歇重叠成 254，
                    # 等它自己消失要 10~15s。处理策略：
                    #   失败≥2 次 → 清图后 0.2s 快速重试；
                    #   失败≥3 次且后方安全 → 物理倒车 0.22m 拉开临界区再重试。
                    self._first_phase_failures += 1
                    if self._first_phase_failures >= 3:
                        rear = self.scan_clearance(False)
                        if rear is not None and rear >= 0.30:
                            self.get_logger().warn(
                                f"[起步拉开] 连续 {self._first_phase_failures} 次规划失败，"
                                f"后方 {rear:.2f}m，倒车 0.22m 拉开后重试"
                            )
                            self.clear_costmaps()
                            self._pullback_ticks = 0
                            self._first_phase_retry_timer = self.create_timer(
                                0.2, self._start_pullback_tick
                            )
                            return
                    if self._first_phase_failures >= 2:
                        self.clear_costmaps()
                    self.get_logger().warn(
                        f"dating.csv 规划 ABORTED，但局部车体代价={local_cost}，"
                        f"失败次数={self._first_phase_failures}，"
                        f"{self.FIRST_PHASE_RETRY_DELAY:.1f}s 后重试"
                    )
                    self._first_phase_retry_timer = self.create_timer(
                        self.FIRST_PHASE_RETRY_DELAY, self.retry_first_phase_route
                    )
                else:
                    # 车体已进 lethal（撞墙/贴死）：死等只会卡住不动。
                    # 恢复策略：清图 + 后方安全则倒车 0.22m 拉开 + 重试；
                    # 后方不足则清图后快速重试（让 Smac 重新规划绕开）。
                    self._first_phase_failures += 1
                    rear = self.scan_clearance(False)
                    if rear is not None and rear >= 0.30:
                        self.get_logger().error(
                            f"dating.csv 规划 ABORTED，局部车体 lethal({local_cost})，"
                            f"后方 {rear:.2f}m，倒车拉开后重试"
                        )
                        self.clear_costmaps()
                        self._pullback_ticks = 0
                        self._first_phase_retry_timer = self.create_timer(
                            0.2, self._start_pullback_tick
                        )
                    else:
                        self.get_logger().error(
                            f"dating.csv 规划 ABORTED，局部车体 lethal({local_cost})"
                            f"且后方不足({rear})，清图后快速重试"
                        )
                        self.clear_costmaps()
                        self._first_phase_retry_timer = self.create_timer(
                            self.FIRST_PHASE_RETRY_DELAY, self.retry_first_phase_route
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
            # 只有真正成功后清零失败计数；重试路径不清零（否则清图/倒车兜底永不触发）
            self._first_phase_failures = 0
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
        completed = self._commit_successful_window()
        if completed == 0:
            self._qr_no_progress_successes += 1
            current = self.amcl_pose
            target = self._active_route_poses[0].pose.position if self._active_route_poses else None
            if current is not None and target is not None:
                distance = math.hypot(target.x - current[0], target.y - current[1])
                self.get_logger().warn(
                    f"[假成功防护] Nav2 连续无进展成功 "
                    f"{self._qr_no_progress_successes}/{self.QR_NO_PROGRESS_SUCCESS_LIMIT} 次，"
                    f"当前 AMCL=({current[0]:.3f},{current[1]:.3f})，"
                    f"目标=({target.x:.3f},{target.y:.3f})，距离={distance:.3f}m"
                )
            if self._qr_no_progress_successes >= self.QR_NO_PROGRESS_SUCCESS_LIMIT:
                self.request_qr_escape(
                    "Nav2 连续返回 SUCCEEDED 但实际未进入完成半径，"
                    "停止重发并执行统一脱困",
                    prefer_reverse=True,
                )
                return
        if self._qr_route_offset < len(self._qr_route_poses):
            self._qr_replan_requested = False
            self.resend_qr_route()
            return
        if not self._qr_final_goal_completed:
            self._qr_route_offset = max(0, len(self._qr_route_poses) - 1)
            self.get_logger().error(
                "[最终回点保护] Nav2 成功但最终目标未确认完成，重新发送最终目标"
            )
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
        self._qr_no_progress_successes = 0
        self._qr_final_goal_completed = False
        self._stall_window.clear()
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
