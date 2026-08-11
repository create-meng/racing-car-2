import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraTriggerPublisher(Node):
    def __init__(self):
        super().__init__('camera_trigger_publisher')
        
        self.publisher_ = self.create_publisher(CompressedImage, 'image', 10)
        self.bridge = CvBridge()
        
        # 打开默认摄像头
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头！请检查设备连接。")
            return
            
        self.get_logger().info("摄像头节点已启动！")
        self.get_logger().info("【提示】请确保选中弹出的摄像头画面窗口，然后按键盘上的 'T' 键捕获图像，按 'Q' 键退出。")

        # 创建定时器，以大约 30 FPS 的频率持续刷新摄像头，防止缓冲区积压旧画面
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        if not self.cap.isOpened():
            return

        # 持续读取，保持帧为最新
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("无法读取摄像头画面", throttle_duration_sec=2.0)
            return

        # 显示实时画面窗口（必须有窗口才能使用 waitKey）
        cv2.imshow("Camera Preview (Press 'T' to Capture, 'Q' to Quit)", frame)
        
        # 监听按键，延迟 1 毫秒
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('t') or key == ord('T'):
            self.get_logger().info("检测到按键 'T'，正在发布图像...")
            self.capture_and_publish(frame)
        elif key == ord('q') or key == ord('Q'):
            self.get_logger().info("检测到退出按键 'Q'，正在关闭...")
            rclpy.shutdown()

    def capture_and_publish(self, frame):
        try:
            # 压缩并转换为 CompressedImage 消息
            msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
            # 发布到话题
            self.publisher_.publish(msg)
            self.get_logger().info("成功发布一帧压缩图像到 'image' 话题！")
        except Exception as e:
            self.get_logger().error(f"图像压缩或发布失败: {str(e)}")

    def destroy_node(self):
        # 释放摄像头并关闭窗口
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
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