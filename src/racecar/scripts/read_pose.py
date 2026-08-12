#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class PoseReader(Node):
    def __init__(self):
        super().__init__('pose_reader')
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.get_logger().info(f'x={x:.3f}  y={y:.3f}  yaw={yaw:.3f} rad（{math.degrees(yaw):.1f}°）')


def main(args=None):
    rclpy.init(args=args)
    node = PoseReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()