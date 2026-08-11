import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class CameraTriggerPublisher(Node):
    def __init__(self):
        super().__init__('camera_trigger_publisher')
        
        # 创建图像发布者
        self.publisher_ = self.create_publisher(CompressedImage, 'image', 10)
        
        # 创建字符串订阅者，监听特殊触发话题
        self.subscription = self.create_subscription(
            String,
            '/special_goal_topic',
            self.trigger_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.latest_frame = None  # 用于缓存摄像头的最新一帧画面
        
        # 打开默认摄像头
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头！请检查设备连接。")
            return
            
        self.get_logger().info("摄像头节点已无头(Headless)模式启动！")
        self.get_logger().info("【提示】等待接收话题 '/special_goal_topic' 的消息以触发截图...")

        # 创建定时器，以大约 30 FPS 的频率持续刷新摄像头缓存
        # 这一步非常关键：即使不显示画面，也必须不断读取，否则摄像头缓冲区会积压旧画面
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        if not self.cap.isOpened():
            return

        # 持续读取，更新最新帧
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("无法读取摄像头画面", throttle_duration_sec=2.0)
            return

        self.latest_frame = frame

    def trigger_callback(self, msg):
        # 接收到目标话题内容时的处理逻辑
        self.get_logger().info(f"接收到触发指令: '{msg.data}'，正在截取图像...")
        
        if self.latest_frame is not None:
            self.capture_and_publish(self.latest_frame)
        else:
            self.get_logger().warn("尚未获取到有效的摄像头画面，无法截取！")

    def capture_and_publish(self, frame):
        try:
            # 压缩并转换为 CompressedImage 消息
            msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
            # 发布到话题
            self.publisher_.publish(msg)
            self.get_logger().info("✅ 成功发布一帧最新压缩图像到 'image' 话题！")
        except Exception as e:
            self.get_logger().error(f"图像压缩或发布失败: {str(e)}")

    def destroy_node(self):
        # 释放摄像头资源
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraTriggerPublisher()
    
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