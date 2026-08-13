import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time


class CameraTriggerPublisher(Node):
    """桥接：从深度相机RGB到image话题

    订阅 Aurora 930 的 rgb/image_raw，缓存最新帧。
    收到 /special_goal_topic 触发时，将缓存帧压缩后发布到 image 话题。
    不再直接使用摄像头设备，不与 saoma.py 冲突。
    """

    def __init__(self):
        super().__init__('camera_trigger_publisher')

        # 发布图像到 image 话题（供AI图生文、网页端等使用）
        self.publisher_ = self.create_publisher(CompressedImage, 'image', 10)

        # 订阅深度相机RGB
        self.create_subscription(Image, '/aurora/rgb/image_raw', self.depth_rgb_callback, 10)

        # 订阅触发话题
        self.create_subscription(String, '/special_goal_topic', self.trigger_callback, 10)

        self.bridge = CvBridge()
        self.latest_frame = None

        self.get_logger().info("深度相机桥接节点已启动")
        self.get_logger().info("等待接收深度相机RGB帧和 /special_goal_topic 触发...")

    def depth_rgb_callback(self, msg):
        """缓存深度相机RGB最新帧"""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f"深度相机帧处理失败: {e}")

    def trigger_callback(self, msg):
        """接收到触发指令 → 截取图像并发布"""
        self.get_logger().info(f"接收到触发指令: '{msg.data}'，正在截取图像...")

        if self.latest_frame is not None:
            self.capture_and_publish(self.latest_frame)
        else:
            self.get_logger().warn("尚未获取到有效的深度相机画面，无法截取！")

    def capture_and_publish(self, frame):
        try:
            msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
            self.publisher_.publish(msg)
            self.get_logger().info("✅ 成功发布一帧深度相机图像到 'image' 话题！")
        except Exception as e:
            self.get_logger().error(f"图像压缩或发布失败: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraTriggerPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()