import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time  # 用于摄像头打开重试时的等待

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
        
        # 打开默认摄像头（优先 V4L2 后端，避免 GStreamer 慢/报错）
        # 摄像头可能刚被 saoma.py 释放，等待并多次重试
        self.cap = self.open_camera()
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头！请检查设备连接。")
            return
            
        self.get_logger().info("摄像头节点已无头(Headless)模式启动！")
        self.get_logger().info("【提示】等待接收话题 '/special_goal_topic' 的消息以触发截图...")

        # 创建定时器，以大约 30 FPS 的频率持续刷新摄像头缓存
        # 这一步非常关键：即使不显示画面，也必须不断读取，否则摄像头缓冲区会积压旧画面
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def open_camera(self):
        """打开摄像头：优先 V4L2 后端，重试多次；失败则回退默认后端（GStreamer）"""
        # 1) 等待摄像头设备被释放（可能刚被 saoma.py 释放）
        time.sleep(1.0)

        # 2) 优先 V4L2 后端，最多重试 3 次
        for attempt in range(1, 4):
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            if cap is not None and cap.isOpened():
                self.get_logger().info(f"已通过 V4L2 后端打开摄像头（第 {attempt} 次尝试）")
                return cap
            if cap is not None:
                cap.release()
            self.get_logger().warn(f"V4L2 后端打开失败（第 {attempt}/3 次），等待 1 秒后重试...")
            time.sleep(1.0)

        # 3) V4L2 全部失败，回退默认后端（GStreamer）
        self.get_logger().warn("V4L2 后端多次尝试失败，改用默认后端（GStreamer）...")
        return cv2.VideoCapture(0)

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