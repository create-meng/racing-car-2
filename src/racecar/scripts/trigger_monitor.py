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

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)
from nav2_msgs.msg import Costmap as Nav2Costmap
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from sensor_msgs.msg import LaserScan

LOG_DIR = "/root/ros2_ws/logs"


class TriggerMonitor(Node):
    """诊断监控节点：旁观 Nav2 话题，检测异常并写入日志文件。"""

    def __init__(self):
        super().__init__("trigger_monitor")

        # ---- 日志文件 ----
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(LOG_DIR, f"trigger_diag_{ts}.log")
        self.log_fh = open(self.log_path, "w", encoding="utf-8")

        # ---- QoS ----
        # 注意：Nav2 的 /costmap_raw、/costmap 发布 QoS 是 transient_local + reliable，
        # 订阅方必须用 TRANSIENT_LOCAL 才能收到（VOLATILE 会收不到，导致 g_cost/near_obs 为空）
        # TRANSIENT_LOCAL 订阅兼容 volatile 发布者，因此 amcl_pose/cmd_vel/goal_pose 也能正常收到
        costmap_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ---- 订阅 ----
        try:
            self.create_subscription(
                Nav2Costmap,
                "/global_costmap/costmap_raw",
                self.cb_global_costmap,
                costmap_qos,
            )
            self.create_subscription(
                Nav2Costmap,
                "/local_costmap/costmap_raw",
                self.cb_local_costmap,
                costmap_qos,
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
        self.amcl_pose = None       # (x, y, yaw)
        self.amcl_cov = None        # covariance array (36)
        self.prev_amcl = None       # (x, y, yaw)
        self.prev_amcl_t = None
        self.cmd_vel = (0.0, 0.0)  # (vx, wz)
        self.goal = None            # (x, y) or None
        self.scan_min = None        # 最近激光点距离
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
            "/amcl_pose  /scan  /goal_pose  /cmd_vel"
        )
        self.log("=" * 70)

    # ====================================================================
    # 回调
    # ====================================================================

    def cb_global_costmap(self, msg):
        self.global_costmap = msg

    def cb_local_costmap(self, msg):
        self.local_costmap = msg

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
        valid = [r for r in msg.ranges if math.isfinite(r) and r > 0.05]
        self.scan_min = min(valid) if valid else None

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
        cx = int((x - cm.origin_x) / cm.resolution)
        cy = int((y - cm.origin_y) / cm.resolution)
        if 0 <= cx < cm.meta_width and 0 <= cy < cm.meta_height:
            return cm.data[cy * cm.meta_width + cx]
        return None

    def footprint_costs(self, cm):
        """返回机器人 footprint (4 角 + 中心) 在全局图上的 cost 列表。"""
        if cm is None or self.amcl_pose is None:
            return []
        x, y, yaw = self.amcl_pose
        # footprint: [中心, 左下, 右下, 右上, 左上]
        pts = [(0, 0), (-0.15, -0.10), (-0.15, 0.10),
               (0.25, 0.10), (0.25, -0.10)]
        costs = []
        for px, py in pts:
            rx = x + px * math.cos(yaw) - py * math.sin(yaw)
            ry = y + px * math.sin(yaw) + py * math.cos(yaw)
            c = self.cost_at(cm, rx, ry)
            if c is not None:
                costs.append(c)
        return costs

    def nearest_obstacle_local(self):
        """局部代价地图中最近障碍物距离 (相对于滚动窗中心 = 机器人位置)。"""
        cm = self.local_costmap
        if cm is None:
            return None
        res = cm.resolution
        w, h = cm.meta_width, cm.meta_height
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
                if c == 253 or c == 254:  # lethal / inscribed (不含 unknown 255)
                    d = math.hypot(dx - cx, dy - cy) * res
                    if best is None or d < best:
                        best = d
        return best

    def count_occupied(self, cm):
        """全局 costmap 中占据单元数 (cost >= 250)。"""
        if cm is None:
            return None
        # nav2_msgs/Costmap: 253=inscribed, 254=lethal, 255=unknown
        return sum(1 for v in cm.data if v == 253 or v == 254)

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
        if self.amcl_pose is None:
            self.log("[SUM] 等待 amcl_pose ...")
            return
        x, y, yaw = self.amcl_pose
        g = self.cost_at(self.global_costmap, x, y)
        n = self.nearest_obstacle_local()
        self.log(
            f"[SUM] amcl=({x:.2f},{y:.2f},{math.degrees(yaw):.1f}°) "
            f"g_cost={g if g is not None else '--'} "
            f"near_obs={f'{n:.2f}' if n else '--'} "
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
        self.log("!" * 70)
        self.log(f"---- 触发前状态缓冲 (最近 {len(self.buffer)} 条) ----")
        for i, e in enumerate(self.buffer):
            self.log(
                f"  [{i:3d}] t={e['t']} "
                f"amcl={e['amcl']} "
                f"g_cost={e['g_cost']} "
                f"fp_costs={e['g_fp_costs']} "
                f"near_obs={e['near_obs']} "
                f"cmd=({e['cmd'][0]:.2f},{e['cmd'][1]:.2f}) "
                f"cov={e['cov']} "
                f"goal={e['goal']}"
            )
        self.log("---- 缓冲结束 ----")
        self.log("")

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
                f"pose=({x:.2f},{y:.2f},{math.degrees(yaw):.1f}°)",
            )
        elif mx == 253:
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