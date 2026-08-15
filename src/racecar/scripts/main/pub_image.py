#!/usr/bin/env python3
"""
图生文图像源节点（深度相机版）

- 图像来源：深度相机（Aurora930）RGB 流 /aurora/rgb/image_raw（BGR888）
- 触发方式：收到 /special_goal_topic 消息后，截取最新一帧深度相机画面，
  保存到 /root/ros2_ws/logs/ 并压缩发布到 'image' 话题，
  供 ai_qw.py（大模型图生文）消费
- 不再占用 USB 摄像头，因此与二维码扫码节点（saoma.py）无设备冲突
"""
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


class DepthImageTriggerPublisher(Node):
    def __init__(self):
        super().__init__('depth_image_trigger_publisher')

        # 兼容旧启动命令中的参数（图生文已改用深度相机，USB camera_index 不再使用）
        self.declare_parameter('camera_index', -1)

        # 创建图像发布者（ai_qw.py 订阅）
        self.publisher_ = self.create_publisher(CompressedImage, 'image', 10)

        # 创建字符串订阅者，监听图生文触发话题
        self.subscription = self.create_subscription(
            String,
            '/special_goal_topic',
            self.trigger_callback,
            10,
        )

        # 订阅深度相机 RGB 流（Aurora930 驱动节点，namespace=aurora）
        # 使用传感器 QoS（best effort），兼容驱动发布端的 QoS
        self.image_sub = self.create_subscription(
            Image,
            '/aurora/rgb/image_raw',
            self.image_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.bridge = CvBridge()
        self.latest_frame = None  # 缓存深度相机最新一帧画面

        self.get_logger().info("图生文图像节点已启动（深度相机 /aurora/rgb/image_raw）")
        self.get_logger().info("【提示】等待接收话题 '/special_goal_topic' 的消息以触发截图...")

    def image_callback(self, msg):
        """持续刷新深度相机最新帧缓存。"""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"深度图像转换失败: {e}", throttle_duration_sec=2.0)

    def trigger_callback(self, msg):
        """接收到图生文触发指令时，截取最新一帧深度相机画面，保存并发布。"""
        self.get_logger().info(f"接收到触发指令: '{msg.data}'，正在截取深度相机图像...")

        if self.latest_frame is not None:
            self.save_snapshot(self.latest_frame)   # 保存截图到日志目录
            self.capture_and_publish(self.latest_frame)
        else:
            self.get_logger().warn("尚未获取到深度相机画面，无法截取！")

    def save_snapshot(self, frame):
        """把截得的图保存到 /root/ros2_ws/logs/，便于赛后查看图生文输入。"""
        log_dir = "/root/ros2_ws/logs"
        try:
            os.makedirs(log_dir, exist_ok=True)
            ts = time.strftime('%Y%m%d_%H%M%S')
            path = os.path.join(log_dir, f'img2text_{ts}.jpg')
            ok = cv2.imwrite(path, frame)
            if ok:
                self.get_logger().info(f"📷 截图已保存: {path}")
                return path
            self.get_logger().error(f"截图保存失败: {path}")
        except Exception as e:
            self.get_logger().error(f"截图保存异常: {e}")
        return None

    def capture_and_publish(self, frame):
        try:
            # 压缩并转换为 CompressedImage 消息
            msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
            # 发布到话题
            self.publisher_.publish(msg)
            self.get_logger().info("✅ 成功发布一帧深度相机图像到 'image' 话题！")
        except Exception as e:
            self.get_logger().error(f"图像压缩或发布失败: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthImageTriggerPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 确保 rclpy 仅在有效时关闭
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
