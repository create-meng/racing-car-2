#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import numpy as np
from math import sin, cos, atan2, pi, fabs

class LaserLocalization(Node):
    def __init__(self):
        super().__init__('laser_localization_node')

        # 订阅
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # 雷达参数
        self.min_dist = 0.1
        self.max_dist = 5.0
        self.back_angle_start = 160 * pi / 180
        self.back_angle_end = 200 * pi / 180
        self.left_angle_start = 80 * pi / 180
        self.left_angle_end = 100 * pi / 180

        # 输出
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0

        # IMU 防抖逻辑（你要的最终版）
        self.last_raw_yaw = None  # 上一帧原始角度
        self.THRESHOLD = 0.01     # 变化 ≤0.01 不更新

    def imu_callback(self, imu_msg):
        # 四元数转角度
        q = imu_msg.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        current_yaw = atan2(siny, cosy)
        current_deg = current_yaw * 180 / pi

        # 初始化
        if self.last_raw_yaw is None:
            self.last_raw_yaw = current_deg
            return

        # 计算相邻帧变化量
        delta = current_deg - self.last_raw_yaw
        self.last_raw_yaw = current_deg

        # 只有变化 >0.01 才累加
        if fabs(delta) > self.THRESHOLD:
            self.car_yaw += delta

        # 保留两位小数
        self.car_yaw = round(self.car_yaw, 2)

    def scan_callback(self, scan_msg):
        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        angles = angle_min + np.arange(len(ranges)) * angle_inc

        # 过滤无效点
        valid_mask = np.isfinite(ranges) & (ranges > self.min_dist) & (ranges < self.max_dist)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        if len(valid_ranges) < 10:
            print(f"0.000 0.000 {self.car_yaw:.2f}")
            return

        # 提取后墙、左墙点云
        back_points = self.extract_points(valid_angles, valid_ranges, self.back_angle_start, self.back_angle_end)
        left_points = self.extract_points(valid_angles, valid_ranges, self.left_angle_start, self.left_angle_end)

        if len(back_points) < 5 or len(left_points) < 5:
            print(f"0.000 0.000 {self.car_yaw:.2f}")
            return

        # 拟合直线（已修复BUG！）
        back_k, back_b = self.fit_line(back_points)
        left_k, left_b = self.fit_line(left_points)

        # 计算距离
        self.car_x = fabs(self.distance_to_line(0, 0, back_k, back_b))
        self.car_y = fabs(self.distance_to_line(0, 0, left_k, left_b))

        # 输出
        print(f"{self.car_x:.3f} {self.car_y:.3f} {self.car_yaw:.2f}")

    def extract_points(self, angles, ranges, a_start, a_end):
        mask = (angles >= a_start) & (angles <= a_end)
        pts = []
        for a, d in zip(angles[mask], ranges[mask]):
            x = d * cos(a)
            y = d * sin(a)
            pts.append([x, y])
        return np.array(pts)

    def fit_line(self, points):
        x = points[:, 0]
        y = points[:, 1]  # 这里已修复！
        A = np.vstack([x, np.ones(len(x))]).T
        k, b = np.linalg.lstsq(A, y, rcond=None)[0]
        return k, b

    def distance_to_line(self, x0, y0, k, b):
        return fabs(k * x0 - y0 + b) / np.sqrt(k**2 + 1)

def main(args=None):
    rclpy.init(args=args)
    node = LaserLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
