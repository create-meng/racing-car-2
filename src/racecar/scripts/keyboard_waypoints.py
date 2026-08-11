#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import csv
import threading
import sys
import termios
import tty

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        # 订阅键盘控制速度
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # 订阅里程计（核心取坐标）
        self.current_pose = None
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # 订阅IMU（你要求包含，已兼容）
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        # 保存路径
        self.save_path = '/root/ros2_ws/src/racecar/scripts/waypoints.csv'
        print("==== 点位记录器已启动 ====")
        print("订阅话题：/cmd_vel /imu/data_raw /odom_combined")
        print("操作：ENTER 保存点位 | Q 退出")
        print("保存格式：x y z w")

        # 启动键盘监听
        threading.Thread(target=self.key_listener, daemon=True).start()

    def cmd_vel_cb(self, msg):
        pass

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose

    def imu_cb(self, msg):
        pass

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def key_listener(self):
        while True:
            key = self.get_key()
            if key == 'q':
                print("退出程序")
                rclpy.shutdown()
                break
            elif key in ['\n', '\r']:
                self.save_waypoint()

    def save_waypoint(self):
        if self.current_pose is None:
            print("等待里程计数据...")
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        z = self.current_pose.position.z
        w = self.current_pose.orientation.w

        with open(self.save_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([x, y, z, w])

        print(f"保存点位：x={x:.2f}, y={y:.2f}, z={z:.2f}, w={w:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()