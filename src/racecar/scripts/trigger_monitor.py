#!/usr/bin/env python3
"""
trigger_monitor.py — 导航"乱飘"触发诊断监控节点
====================================================
旁观 Nav2 运行时的关键话题，检测异常并写出详细日志。
不改动任何 Nav2 配置，只做日志记录。

运行方式（在导航启动后，另开终端）:
    cd /root/ros2_ws
    python3 src/racecar/scripts/trigger_monitor.py

日志输出:
    /root/ros2_ws/logs/trigger_diag_<YYYYMMDD_HHMMSS>.log

检测的异常(触发器):
    1. HIGH   — 机器人 in footprint 在全局代价地图进入 lethal (raw cost >= 254)
    2. HIGH   — AMCL 位姿跳变 (>0.8m 或远超指令速度能到达的范围)
    3. HIGH   — 全局代价地图被清空 (占据单元数骤降 >50%)
    4. MEDIUM — 附近突然出现新障碍 (最近障碍距离骤降 >1.0m)
    5. MEDIUM — AMCL 协方差飙高 (3 倍以上增幅)
"""

import os
import sys
import time
import math
import datetime
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import Costmap
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

LOG_DIR = "/root/ros2_ws/logs"
SNAPSHOT_DIR = os.path.join(LOG_DIR, "trigger_snapshots")


class TriggerMonitor(Node):
    """诊断监控节点：旁观 Nav2 话题，检测异常并写入日志文件。"""

    def __init__(self):
        super().__init__("trigger_monitor")

        # ---- 日志文件 ----
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(LOG_DIR, f"trigger_diag_{ts}.log")
        self.log_fh = open(self.log_path, "w", encoding="utf-8")
        os.makedirs(SNAPSHOT_DIR, exist_ok=True)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- QoS ----
        costmap_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ---- 订阅 ----
        try:
            # costmap_raw 使用 nav2_msgs/Costmap，保留 0-255 原始代价。
            self.create_subscription(
                Costmap, "/global_costmap/costmap_raw", self.cb_global_costmap, costmap_qos)
            self.create_subscription(
                Costmap, "/local_costmap/costmap_raw", self.cb_local_costmap, costmap_qos)
            self.create_subscription(
                OccupancyGrid,
                "/map",
                self.cb_static_map,
                map_qos,
            )
            self.create_subscription(
                PoseWithCovarianceStamped,
                "/amcl_pose",
                self.cb_amcl,
                costmap_qos,
            )
            self.create_subscription(
                LaserScan,
                "/scan",
                self.cb_scan,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Odometry,
                "/odom_combined",
                self.cb_odom,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                PoseStamped,
                "/goal_pose",
                self.cb_goal,
                costmap_qos,
            )
            self.create_subscription(
                Twist,
                "/cmd_vel",
                self.cb_cmd,
                costmap_qos,
            )
        except Exception as e:
            self.log(f"[ERROR] 订阅创建失败: {e}")
            raise

        # ---- 状态 ----
        self.global_costmap = None  # nav2_msgs/Costmap (raw 0-255)
        self.local_costmap = None   # nav2_msgs/Costmap (raw 0-255)
        self.static_map = None      # nav_msgs/OccupancyGrid (0-100 / -1)
        self.amcl_pose = None       # (x, y, yaw)
        self.amcl_cov = None        # covariance array (36)
        self.prev_amcl = None       # (x, y, yaw)
        self.prev_amcl_t = None
        self.cmd_vel = (0.0, 0.0)  # (vx, wz)
        self.goal = None            # (x, y) or None
        self.scan_min = None        # 最近激光点距离
        self.scan_nearest = None    # (距离, 雷达角度 rad)
        self.scan_sector_min = {}   # 前/左/后/右扇区最近距离
        self.scan_age = None        # 最新激光相对本机 ROS 时钟的年龄
        self.scan_frame = "--"
        self.scan_valid = 0
        self.scan_count = 0
        self.latest_scan = None
        self.scan_last_wall = None
        self.scan_period = None
        self.odom_count = 0
        self.odom_last_wall = None
        self.odom_age = None
        self.odom_frame = "--"
        self.odom_child_frame = "--"
        self.global_costmap_count = 0
        self.local_costmap_count = 0
        self.global_costmap_last_wall = None
        self.local_costmap_last_wall = None
        self.global_costmap_period = None
        self.local_costmap_period = None
        self.prev_near_obs = None   # 上一次最近障碍距离
        self.prev_cov = None        # 上一次协方差大小
        self.global_occ_counts = []  # 近 20 次全局 costmap 占据数
        self.buffer = []             # 滚动状态缓冲 (最近 100 条)
        self.last_trigger_t = {}     # 各触发器冷却时间戳
        self.COOLDOWN = 10.0         # 秒

        # ---- 定时器 ----
        self.create_timer(0.5, self.periodic_snapshot)  # 2 Hz 状态快照
        self.create_timer(5.0, self.periodic_summary)   # 每 5s 一行摘要

        # ---- 启动 ----
        self.log("=" * 70)
        self.log(
            f"trigger_monitor 启动 | 日志: {self.log_path}"
        )
        self.log(
            "订阅: /global_costmap/costmap_raw  /local_costmap/costmap_raw  "
            "/map  /amcl_pose  /scan  /goal_pose  /cmd_vel"
        )
        self.log("=" * 70)

    # ====================================================================
    # 回调
    # ====================================================================

    def cb_global_costmap(self, msg):
        self.global_costmap = msg
        now = time.monotonic()
        if self.global_costmap_last_wall is not None:
            self.global_costmap_period = now - self.global_costmap_last_wall
        self.global_costmap_last_wall = now
        self.global_costmap_count += 1

    def cb_local_costmap(self, msg):
        self.local_costmap = msg
        now = time.monotonic()
        if self.local_costmap_last_wall is not None:
            self.local_costmap_period = now - self.local_costmap_last_wall
        self.local_costmap_last_wall = now
        self.local_costmap_count += 1

    def cb_static_map(self, msg):
        self.static_map = msg

    def cb_amcl(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.amcl_pose = (p.position.x, p.position.y, yaw)
        self.amcl_cov = msg.pose.covariance
        self.check_amcl_jump()

    def cb_scan(self, msg):
        self.latest_scan = msg
        valid = [
            (r, msg.angle_min + i * msg.angle_increment)
            for i, r in enumerate(msg.ranges)
            if math.isfinite(r) and r > 0.05
        ]
        if valid:
            self.scan_nearest = min(valid)
            self.scan_min = self.scan_nearest[0]
            sectors = {"front": [], "left": [], "rear": [], "right": []}
            for distance, angle in valid:
                angle = math.atan2(math.sin(angle), math.cos(angle))
                if -math.pi / 4 <= angle < math.pi / 4:
                    sectors["front"].append(distance)
                elif math.pi / 4 <= angle < 3 * math.pi / 4:
                    sectors["left"].append(distance)
                elif -3 * math.pi / 4 <= angle < -math.pi / 4:
                    sectors["right"].append(distance)
                else:
                    sectors["rear"].append(distance)
            self.scan_sector_min = {
                name: min(values) if values else None for name, values in sectors.items()
            }
        else:
            self.scan_min = None
            self.scan_nearest = None
            self.scan_sector_min = {}
        now_wall = time.monotonic()
        if self.scan_last_wall is not None:
            self.scan_period = now_wall - self.scan_last_wall
        self.scan_last_wall = now_wall
        self.scan_count += 1
        self.scan_valid = len(valid)
        self.scan_frame = msg.header.frame_id or "<empty>"
        stamp = msg.header.stamp
        if stamp.sec or stamp.nanosec:
            now = self.get_clock().now().nanoseconds
            self.scan_age = (now - stamp.sec * 1_000_000_000 - stamp.nanosec) / 1_000_000_000

    def cb_odom(self, msg):
        self.odom_count += 1
        self.odom_last_wall = time.monotonic()
        self.odom_frame = msg.header.frame_id or "<empty>"
        self.odom_child_frame = msg.child_frame_id or "<empty>"
        stamp = msg.header.stamp
        if stamp.sec or stamp.nanosec:
            now = self.get_clock().now().nanoseconds
            self.odom_age = (now - stamp.sec * 1_000_000_000 - stamp.nanosec) / 1_000_000_000

    def cb_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.log(f"[GOAL] 收到新目标: ({self.goal[0]:.3f}, {self.goal[1]:.3f})")

    def cb_cmd(self, msg):
        self.cmd_vel = (msg.linear.x, msg.angular.z)

    # ====================================================================
    # 工具
    # ====================================================================

    def log(self, text):
        """写入日志文件 + 终端。"""
        line = f"[{datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {text}"
        print(line, flush=True)
        self.log_fh.write(line + "\n")
        self.log_fh.flush()

    def cost_at(self, cm, x, y):
        """在 map 系取 (x, y) 处的 raw cost (0-255)。"""
        if cm is None:
            return None
        info = cm.metadata if isinstance(cm, Costmap) else cm.info
        o = info.origin.position
        cx = int((x - o.x) / info.resolution)
        cy = int((y - o.y) / info.resolution)
        width = info.size_x if isinstance(cm, Costmap) else info.width
        height = info.size_y if isinstance(cm, Costmap) else info.height
        if 0 <= cx < width and 0 <= cy < height:
            return cm.data[cy * width + cx]
        return None

    def footprint_costs(self, cm):
        """返回当前矩形足迹覆盖的全部代价格。"""
        if cm is None or self.amcl_pose is None:
            return []
        x, y, yaw = self.amcl_pose
        info = cm.metadata if isinstance(cm, Costmap) else cm.info
        resolution = info.resolution
        costs = []
        for px in self._frange(-0.15, 0.15, resolution):
            for py in self._frange(-0.12, 0.12, resolution):
                rx = x + px * math.cos(yaw) - py * math.sin(yaw)
                ry = y + px * math.sin(yaw) + py * math.cos(yaw)
                c = self.cost_at(cm, rx, ry)
                if c is not None:
                    costs.append(c)
        return costs

    @staticmethod
    def _frange(start, stop, step):
        value = start + step / 2.0
        while value <= stop:
            yield value
            value += step

    def nearest_obstacle_local(self):
        """局部代价地图中最近障碍物距离 (相对于滚动窗中心 = 机器人位置)。"""
        cm = self.local_costmap
        if cm is None:
            return None
        res = cm.metadata.resolution
        w, h = cm.metadata.size_x, cm.metadata.size_y
        cx, cy = w // 2, h // 2
        best = None
        r_cells = int(5.0 / res)  # 只扫 5m 半径
        y0 = max(0, cy - r_cells)
        y1 = min(h, cy + r_cells)
        x0 = max(0, cx - r_cells)
        x1 = min(w, cx + r_cells)
        for dy in range(y0, y1):
            row_start = dy * w
            for dx in range(x0, x1):
                c = cm.data[row_start + dx]
                if c >= 250:  # lethal / inscribed
                    d = math.hypot(dx - cx, dy - cy) * res
                    if best is None or d < best:
                        best = d
        return best

    def count_occupied(self, cm):
        """全局 costmap 中占据单元数 (cost >= 250)。"""
        if cm is None:
            return None
        return sum(1 for v in cm.data if v >= 250)

    def cov_magnitude(self):
        """AMCL 协方差矩阵的 x, y, yaw 对角元平方和。"""
        if self.amcl_cov is None:
            return None
        # covariance[0] = x, [7] = y, [35] = yaw
        return math.sqrt(self.amcl_cov[0] ** 2 + self.amcl_cov[7] ** 2 + self.amcl_cov[35] ** 2)

    # ====================================================================
    # 状态快照 (每 0.5s)
    # ====================================================================

    def periodic_snapshot(self):
        entry = {
            "t": datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "amcl": self.amcl_pose,
            "cov": self.cov_magnitude(),
            "goal": self.goal,
            "cmd": self.cmd_vel,
            "scan_min": self.scan_min,
            "scan_nearest": self.scan_nearest,
            "scan_sector_min": self.scan_sector_min,
        }

        # 全局图机器人 cost
        if self.amcl_pose is not None:
            g = self.cost_at(self.global_costmap, self.amcl_pose[0], self.amcl_pose[1])
            fp = self.footprint_costs(self.global_costmap)
        else:
            g = None
            fp = []
        entry["g_cost"] = g
        entry["g_fp_costs"] = fp
        entry["static_fp_costs"] = self.footprint_costs(self.static_map)
        entry["scan_age"] = self.scan_age
        entry["scan_frame"] = self.scan_frame
        entry["scan_valid"] = self.scan_valid
        entry["scan_count"] = self.scan_count
        entry["scan_period"] = self.scan_period
        entry["global_cm_count"] = self.global_costmap_count
        entry["local_cm_count"] = self.local_costmap_count

        # 局部图最近障碍
        near_obs = self.nearest_obstacle_local()
        entry["near_obs"] = near_obs

        self.buffer.append(entry)
        if len(self.buffer) > 100:
            self.buffer.pop(0)

        # ---- 运行检测器 ----
        self.check_lethal_global()
        self.check_costmap_cleared()
        self.check_obs_spawn(near_obs)
        self.check_cov_spike()

    def periodic_summary(self):
        now = time.monotonic()
        scan_silent = self.scan_last_wall is None or now - self.scan_last_wall > 1.0
        local_cm_silent = self.local_costmap_last_wall is None or now - self.local_costmap_last_wall > 1.0
        if scan_silent:
            self.log(f"[LINK][HIGH] /scan 未收到或已中断: rx={self.scan_count} last={self.scan_last_wall}")
        if local_cm_silent:
            self.log(f"[LINK][HIGH] /local_costmap/costmap 未收到或已中断: rx={self.local_costmap_count} last={self.local_costmap_last_wall}")
        if self.amcl_pose is None:
            self.log(
                "[SUM] 等待 amcl_pose | "
                f"publishers=(amcl:{self.count_publishers('/amcl_pose')},scan:{self.count_publishers('/scan')}) "
                f"scan_rx={self.scan_count} age={self._age_text(self.scan_age)} frame={self.scan_frame} "
                f"odom_rx={self.odom_count} age={self._age_text(self.odom_age)} "
                f"frames={self.odom_frame}->{self.odom_child_frame} "
                f"tf_now(odom->base_footprint)={self._transform_status('odom_combined', 'base_footprint')} "
                f"tf_scan(odom->base_footprint)={self._scan_transform_status()} "
                f"tf(base_footprint->laser)={self._transform_status('base_footprint', 'laser')}"
            )
            return
        x, y, yaw = self.amcl_pose
        g = self.cost_at(self.global_costmap, x, y)
        n = self.nearest_obstacle_local()
        nearest = "--" if self.scan_nearest is None else (
            f"{self.scan_nearest[0]:.2f}m@{math.degrees(self.scan_nearest[1]):.1f}deg"
        )
        sectors = ",".join(
            f"{name}:{distance:.2f}" if distance is not None else f"{name}:--"
            for name, distance in self.scan_sector_min.items()
        ) or "--"
        self.log(
            f"[SUM] amcl=({x:.2f},{y:.2f},{math.degrees(yaw):.1f}°) "
            f"g_cost={g if g is not None else '--'} "
            f"static_fp={self.footprint_costs(self.static_map)} "
            f"near_obs={f'{n:.2f}' if n else '--'} "
            f"scan_min={f'{self.scan_min:.2f}' if self.scan_min else '--'} "
            f"scan_nearest={nearest} sectors=({sectors}) "
            f"scan_age={f'{self.scan_age:.2f}s' if self.scan_age is not None else '--'} "
            f"scan_frame={self.scan_frame} valid={self.scan_valid} "
            f"scan_rx={self.scan_count} period={f'{self.scan_period:.3f}s' if self.scan_period else '--'} "
            f"costmap_rx=(g:{self.global_costmap_count},l:{self.local_costmap_count}) "
            f"costmap_period=(g:{f'{self.global_costmap_period:.3f}s' if self.global_costmap_period else '--'},"
            f"l:{f'{self.local_costmap_period:.3f}s' if self.local_costmap_period else '--'}) "
            f"cmd=({self.cmd_vel[0]:.2f},{self.cmd_vel[1]:.2f})"
        )

    # ====================================================================
    # 异常检测器
    # ====================================================================

    def trigger(self, tag, level, msg):
        """触发异常记录：写入日志并转储滚动缓冲。"""
        now = time.time()
        if now - self.last_trigger_t.get(tag, 0) < self.COOLDOWN:
            return
        self.last_trigger_t[tag] = now

        self.log("")
        self.log("!" * 70)
        self.log(f"[TRIGGER][{tag}][{level}] {msg}")
        if tag == "LETHAL":
            self.save_lethal_snapshot()
        self.log("!" * 70)
        self.log(f"---- 触发前状态缓冲 (最近 {len(self.buffer)} 条) ----")
        for i, e in enumerate(self.buffer):
            self.log(
                f"  [{i:3d}] t={e['t']} "
                f"amcl={e['amcl']} "
                f"g_cost={e['g_cost']} "
                f"fp_costs={e['g_fp_costs']} "
                f"static_fp={e['static_fp_costs']} "
                f"near_obs={e['near_obs']} "
                f"scan_nearest={e['scan_nearest']} sectors={e['scan_sector_min']} "
                f"scan_age={e['scan_age']} "
                f"scan_frame={e['scan_frame']} valid={e['scan_valid']} "
                f"scan_rx={e['scan_count']} period={e['scan_period']} "
                f"costmap_rx=(g:{e['global_cm_count']},l:{e['local_cm_count']}) "
                f"cmd=({e['cmd'][0]:.2f},{e['cmd'][1]:.2f}) "
                f"cov={e['cov']} "
                f"goal={e['goal']}"
            )
        self.log("---- 缓冲结束 ----")
        self.log("")

    @staticmethod
    def _costmap_json(cm):
        if cm is None:
            return None
        info = cm.metadata
        return {
            "frame_id": cm.header.frame_id,
            "stamp": {"sec": cm.header.stamp.sec, "nanosec": cm.header.stamp.nanosec},
            "resolution": info.resolution,
            "width": info.size_x,
            "height": info.size_y,
            "origin": [info.origin.position.x, info.origin.position.y],
            "data": list(cm.data),
        }

    def _transform_json(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            return {"target": target, "source": source, "translation": [t.x, t.y, t.z],
                    "rotation": [q.x, q.y, q.z, q.w]}
        except TransformException as exc:
            return {"target": target, "source": source, "error": str(exc)}

    def _transform_status(self, target, source):
        try:
            self.tf_buffer.lookup_transform(target, source, Time())
            return "OK"
        except TransformException as exc:
            return f"ERROR:{str(exc)[:160]}"

    def _scan_transform_status(self):
        if self.latest_scan is None:
            return "NO_SCAN"
        try:
            self.tf_buffer.lookup_transform(
                "odom_combined",
                "base_footprint",
                Time.from_msg(self.latest_scan.header.stamp),
            )
            return "OK"
        except TransformException as exc:
            return f"ERROR:{str(exc)[:160]}"

    @staticmethod
    def _age_text(age):
        return "--" if age is None else f"{age:.3f}s"

    def save_lethal_snapshot(self):
        """首次 lethal 时保存原始数据，供事后复现障碍来源。"""
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        scan = self.latest_scan
        snapshot = {
            "reason": "global footprint contains cost >= 254",
            "wall_time": datetime.datetime.now().isoformat(),
            "amcl_pose": self.amcl_pose,
            "cmd_vel": self.cmd_vel,
            "global_costmap_raw": self._costmap_json(self.global_costmap),
            "local_costmap_raw": self._costmap_json(self.local_costmap),
            "scan": None if scan is None else {
                "frame_id": scan.header.frame_id,
                "stamp": {"sec": scan.header.stamp.sec, "nanosec": scan.header.stamp.nanosec},
                "angle_min": scan.angle_min,
                "angle_increment": scan.angle_increment,
                "range_min": scan.range_min,
                "range_max": scan.range_max,
                "ranges": [r if math.isfinite(r) else None for r in scan.ranges],
            },
            "transforms": [
                self._transform_json("map", "base_footprint"),
                self._transform_json("base_link", "laser"),
            ],
        }
        path = os.path.join(SNAPSHOT_DIR, f"lethal_{ts}.json")
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(snapshot, fh, ensure_ascii=False, allow_nan=False)
        self.log(f"[SNAPSHOT] lethal 原始数据已保存: {path}")

    def check_lethal_global(self):
        """检测器 1: 机器人 footprint 在全局图上进入 lethal (>=254)。"""
        if self.amcl_pose is None or self.global_costmap is None:
            return
        costs = self.footprint_costs(self.global_costmap)
        if not costs:
            return
        mx = max(costs)
        if mx >= 254:
            x, y, yaw = self.amcl_pose
            self.trigger(
                "LETHAL",
                "HIGH",
                f"机器人 footprint 在全局图进入 lethal! "
                f"cost_max={mx} fp_costs={costs} "
                f"static_fp_costs={self.footprint_costs(self.static_map)} "
                f"scan_min={self.scan_min} scan_age={self.scan_age} "
                f"pose=({x:.2f},{y:.2f},{math.degrees(yaw):.1f}°)",
            )
        elif mx >= 253:
            x, y, _ = self.amcl_pose
            self.trigger(
                "INSCRIBED",
                "MEDIUM",
                f"机器人 footprint 进入 inscribed(253)! "
                f"cost_max={mx} pose=({x:.2f},{y:.2f})",
            )

    def check_amcl_jump(self):
        """检测器 2: 连续 AMCL 位姿突变。"""
        if self.prev_amcl is None:
            self.prev_amcl = self.amcl_pose
            self.prev_amcl_t = time.time()
            return
        if self.amcl_pose is None:
            return
        px, py, pyaw = self.prev_amcl
        x, y, yaw = self.amcl_pose
        dt = time.time() - self.prev_amcl_t
        if dt <= 0:
            return

        dist = math.hypot(x - px, y - py)
        dyaw = abs(((yaw - pyaw + math.pi) % (2 * math.pi)) - math.pi)
        v = abs(self.cmd_vel[0])
        expected = v * dt + 0.25  # 期望移动量 + 余量

        if dist > max(0.8, 3.0 * expected) and dist > 0.5:
            self.trigger(
                "AMCL_JUMP",
                "HIGH",
                f"AMCL 位姿跳变! "
                f"dist={dist:.2f}m (期望<={expected:.2f}m) "
                f"yaw_d={math.degrees(dyaw):.1f}° "
                f"前=({px:.2f},{py:.2f}) 后=({x:.2f},{y:.2f})",
            )
        elif dyaw > 1.2 and dist > 0.3:
            self.trigger(
                "AMCL_YAW_JUMP",
                "MEDIUM",
                f"AMCL 朝向跳变! "
                f"yaw_d={math.degrees(dyaw):.1f}° "
                f"dist={dist:.2f}m "
                f"前=({px:.2f},{py:.2f}) 后=({x:.2f},{y:.2f})",
            )
        self.prev_amcl = self.amcl_pose
        self.prev_amcl_t = time.time()

    def check_costmap_cleared(self):
        """检测器 3: 全局代价地图占据单元数骤降 -> 被清空。"""
        n = self.count_occupied(self.global_costmap)
        if n is None:
            return
        self.global_occ_counts.append(n)
        if len(self.global_occ_counts) > 20:
            self.global_occ_counts.pop(0)
        if len(self.global_occ_counts) >= 3:
            prev_max = max(self.global_occ_counts[-4:-1])
            if prev_max > 200 and n < prev_max * 0.5:
                self.trigger(
                    "COSTMAP_CLEAR",
                    "HIGH",
                    f"全局代价地图占据单元骤降: {prev_max} -> {n} (疑似被清空/恢复)",
                )

    def check_obs_spawn(self, near_obs):
        """检测器 4: 附近突然出现新障碍。"""
        if near_obs is None:
            self.prev_near_obs = None
            return
        if self.prev_near_obs is not None and near_obs < 0.5 and self.prev_near_obs - near_obs > 1.0:
            self.trigger(
                "OBS_SPAWN",
                "MEDIUM",
                f"附近突然出现障碍! 最近障碍距离 {self.prev_near_obs:.2f}m -> {near_obs:.2f}m",
            )
        self.prev_near_obs = near_obs

    def check_cov_spike(self):
        """检测器 5: AMCL 协方差飙高。"""
        m = self.cov_magnitude()
        if m is None:
            self.prev_cov = None
            return
        if self.prev_cov is not None and m > 0.5 and m > self.prev_cov * 3:
            self.trigger(
                "COV_SPIKE",
                "MEDIUM",
                f"AMCL 协方差飙高! {self.prev_cov:.3f} -> {m:.3f}",
            )
        self.prev_cov = m


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TriggerMonitor()
        print(f"\n>>> 监控日志: {node.log_path}\n", flush=True)
        rclpy.spin(node)
    except Exception as e:
        print(f"[trigger_monitor] 崩溃: {type(e).__name__}: {e}", flush=True)
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
