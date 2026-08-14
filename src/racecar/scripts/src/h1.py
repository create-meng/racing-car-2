#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String  # 必须是String！
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from action_msgs.msg import GoalStatus
import csv
import os
import re
import math


class NavThroughPosesClient(Node):
    ROUTE_GOAL_RADIUS = 0.15
    FINAL_RETURN_X = 0.0
    FINAL_RETURN_Y = 0.1
    ROUTE_WAYPOINT_TIMEOUT = 30.0
    ROUTE_NO_PROGRESS_TIMEOUT = 12.0
    ROUTE_PROGRESS_DISTANCE = 0.05

    def __init__(self):
        super().__init__('nav_through_poses_client')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._goal_handle = None # 用于记录当前任务句柄
        self._goal_pending = False  # 首阶段目标已发送，但服务端尚未返回句柄
        self._first_phase_cancel_requested = False
        self._route_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._route_goal_handle = None

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

        self.marker_pub = self.create_publisher(Marker, "/forbidden_zone_marker", 10)
        self.timer = self.create_timer(1.0, self.show_forbidden_zone)

        # 用于发布到达特定点时的文本
        self.text_pub = self.create_publisher(String, "/special_goal_topic", 10)

        # 拍照触发标志与路点统计
        self.photo_triggered = False
        self.total_poses = 0
        self.trigger_point = 0
        self._last_visited = None
        self.first_phase_done = False
        self.route_poses = []
        self.route_index = 0
        self.route_reached = []
        self.route_skipped = []
        self.route_goal_started_at = None
        self.route_last_motion_at = None
        self.route_last_amcl_xy = None
        self.route_canceling = False
        self.route_skip_reason = None
        self.route_last_feedback_at = 0.0
        self.route_watchdog = self.create_timer(1.0, self.route_watchdog_callback)
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

        if not self.first_phase_done:
            if status == GoalStatus.STATUS_CANCELED and self.qr_result is None:
                self.get_logger().error("第一阶段被取消，但未收到二维码，不能按正常完成处理")
                return
            if status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error("第一阶段导航异常失败，未继续切换路线")
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
            self.get_logger().info("奇数：执行完整 shunshizhen1.csv，到达第 15 个点后触发拍照")
            next_wp = self.read_waypoints_from_csv("/root/ros2_ws/src/racecar/scripts/point/shunshizhen1.csv")
            self.trigger_point = 15
        else:
            self.get_logger().info("偶数：执行 test_1.csv，到达第 34 个点后触发拍照")
            next_wp = self.read_waypoints_from_csv("/root/ros2_ws/src/racecar/scripts/point/test_1.csv")
            self.trigger_point = 34

        self.route_poses = [self.make_pose(x, y, z, w) for x, y, z, w in next_wp]
        self.total_poses = len(self.route_poses)
        self.photo_triggered = False
        self.route_index = 0
        self.route_reached = []
        self.route_skipped = []
        self.get_logger().info(
            f"二维码路线逐点导航启动：共 {self.total_poses} 个点，完成半径 "
            f"{self.ROUTE_GOAL_RADIUS:.2f} m"
        )
        self.start_next_route_waypoint()

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

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def start_next_route_waypoint(self):
        if self.route_index >= self.total_poses:
            self.finish_route()
            return

        pose = self.route_poses[self.route_index]
        is_final_return = (
            self.qr_result is not None
            and self.qr_result % 2 == 1
            and self.route_index == self.total_poses - 1
        )
        if is_final_return and self.amcl_pose is not None:
            current_x, current_y, _ = self.amcl_pose
            if 0.0 <= current_x <= 0.5 and 0.0 <= current_y <= 0.3:
                self.get_logger().info(
                    f"[主点 {self.route_index + 1}/{self.total_poses}] "
                    f"AMCL 已在起点完成区域内：x={current_x:.3f} y={current_y:.3f}，"
                    "不再发送补充目标"
                )
                self.complete_route_waypoint()
                return
        if is_final_return:
            pose = self.make_pose(
                self.FINAL_RETURN_X,
                self.FINAL_RETURN_Y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )
            self.get_logger().info(
                f"[主点 {self.route_index + 1}/{self.total_poses}] "
                f"最终回起点目标：x={self.FINAL_RETURN_X:.3f} "
                f"y={self.FINAL_RETURN_Y:.3f}（忽略航向）"
            )
        if not self._route_action_client.wait_for_server(timeout_sec=10.0):
            self.skip_route_waypoint("navigate_to_pose 服务不可用")
            return
        self.route_goal_started_at = self.now_seconds()
        self.route_last_motion_at = self.route_goal_started_at
        self.route_last_amcl_xy = None if self.amcl_pose is None else self.amcl_pose[:2]
        self.route_canceling = False
        self.route_skip_reason = None
        self.route_last_feedback_at = 0.0
        self.get_logger().info(
            f"[主点 {self.route_index + 1}/{self.total_poses}] 开始："
            f"x={pose.pose.position.x:.3f} y={pose.pose.position.y:.3f}"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = "/root/ros2_ws/src/racecar/config/navigate_to_pose_precise.xml"
        send_future = self._route_action_client.send_goal_async(
            goal_msg, feedback_callback=self.route_feedback_callback
        )
        send_future.add_done_callback(self.route_goal_response_callback)

    def route_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.skip_route_waypoint(f"发送目标异常：{exc}")
            return
        if not goal_handle.accepted:
            self.skip_route_waypoint("目标被 Nav2 拒绝")
            return

        self._route_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.route_result_callback)

    def route_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        now = self.now_seconds()
        if self.route_index >= self.total_poses or now - self.route_last_feedback_at < 1.0:
            return
        self.route_last_feedback_at = now
        self.get_logger().info(
            f"[主点 {self.route_index + 1}/{self.total_poses}] "
            f"剩余 {feedback.distance_remaining:.2f} m，恢复 {feedback.number_of_recoveries} 次"
        )

    def route_result_callback(self, future):
        self._route_goal_handle = None
        try:
            status = future.result().status
        except Exception as exc:
            self.skip_route_waypoint(f"获取结果异常：{exc}")
            return

        if self.route_canceling:
            self.route_canceling = False
            self.skip_route_waypoint(self.route_skip_reason or "单点 watchdog 取消")
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.complete_route_waypoint()
            return

        status_names = {
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        self.skip_route_waypoint(f"Nav2 返回 {status_names.get(status, status)}")

    def complete_route_waypoint(self):
        pose = self.route_poses[self.route_index]
        self.route_reached.append(self.route_index)
        self.get_logger().info(
            f"[主点 {self.route_index + 1}/{self.total_poses}] 已到达："
            f"x={pose.pose.position.x:.3f} y={pose.pose.position.y:.3f}"
        )
        if not self.photo_triggered and self.route_index + 1 >= self.trigger_point:
            msg = String()
            msg.data = "gaol"
            self.text_pub.publish(msg)
            self.photo_triggered = True
            self.get_logger().info(f"已到达第 {self.trigger_point} 个点，触发拍照")
        self.route_index += 1
        self.start_next_route_waypoint()

    def skip_route_waypoint(self, reason):
        if self.route_index >= self.total_poses:
            self.finish_route()
            return
        pose = self.route_poses[self.route_index]
        self.route_skipped.append((self.route_index, reason))
        self.get_logger().warn(
            f"[主点 {self.route_index + 1}/{self.total_poses}] 跳过：{reason} | "
            f"x={pose.pose.position.x:.3f} y={pose.pose.position.y:.3f}"
        )
        self.route_index += 1
        self.start_next_route_waypoint()

    def route_watchdog_callback(self):
        if self._route_goal_handle is None or self.route_canceling:
            return

        now = self.now_seconds()
        if self.amcl_pose is not None:
            current_xy = self.amcl_pose[:2]
            if self.route_last_amcl_xy is None or math.dist(current_xy, self.route_last_amcl_xy) >= self.ROUTE_PROGRESS_DISTANCE:
                self.route_last_amcl_xy = current_xy
                self.route_last_motion_at = now

        if now - self.route_goal_started_at >= self.ROUTE_WAYPOINT_TIMEOUT:
            self.cancel_route_waypoint(f"总超时 {self.ROUTE_WAYPOINT_TIMEOUT:.0f}s")
        elif self.amcl_pose is not None and now - self.route_last_motion_at >= self.ROUTE_NO_PROGRESS_TIMEOUT:
            self.cancel_route_waypoint(f"{self.ROUTE_NO_PROGRESS_TIMEOUT:.0f}s 无进展")

    def cancel_route_waypoint(self, reason):
        self.route_canceling = True
        self.route_skip_reason = reason
        self.get_logger().warn(
            f"[主点 {self.route_index + 1}/{self.total_poses}] {reason}，取消并继续后续点"
        )
        cancel_future = self._route_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.route_cancel_response_callback)

    def route_cancel_response_callback(self, future):
        try:
            count = len(future.result().goals_canceling)
            if count == 0:
                self.get_logger().warn("当前主点取消请求未被接受，将等待 Nav2 返回结果")
        except Exception as exc:
            self.get_logger().error(f"当前主点取消请求失败：{exc}")

    def finish_route(self):
        skipped = ", ".join(str(index + 1) for index, _ in self.route_skipped) or "无"
        self.get_logger().info(
            f"二维码路线处理结束：到达 {len(self.route_reached)}/{self.total_poses}，"
            f"跳过 {len(self.route_skipped)} 个（序号：{skipped}）"
        )
        rclpy.shutdown()

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
        else:
            self.get_logger().info(
                "剩余距离：{0:.2f} m | 剩余路点 {1} 个".format(remaining, feedback.number_of_poses_remaining),
                once=True,
            )

        if not self.photo_triggered and self.total_poses > 0:
            if visited >= self.trigger_point:
                msg = String()
                msg.data = "gaol"
                self.text_pub.publish(msg)
                self.photo_triggered = True
                self.get_logger().info(f"已到达第 {self.trigger_point} 个点，触发拍照")

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
    client.send_goal(poses)
    rclpy.spin(client)

if __name__ == '__main__':
    main()
