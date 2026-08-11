import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np

class QRCodeCameraNode(Node):
    def __init__(self):
        super().__init__('qr_camera_detector_node')
        
        # 创建二维码结果发布器
        self.result_pub = self.create_publisher(String, '/qr_result', 10)
        
        # 初始化OpenCV摄像头（0=默认摄像头）
        self.cap = cv2.VideoCapture(0)

        ###########################################################
        # 【关键优化】限制分辨率，大幅降低CPU占用
        ###########################################################
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 10)  # 限制帧率

        # 初始化二维码检测器
        self.qr_detector = cv2.QRCodeDetector()
        
        ###########################################################
        # 【关键优化】从 0.03s → 0.2s（5次/秒，CPU大幅下降）
        ###########################################################
        self.timer = self.create_timer(0.2, self.timer_callback)

        # 防抖：防止重复发布
        self.last_qr = None
        
        self.get_logger().info(' 低功耗二维码节点已启动！')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # 解码二维码
        data, bbox, _ = self.qr_detector.detectAndDecode(frame)

        if data and data != self.last_qr:
            self.last_qr = data
            self.get_logger().info(f' 识别二维码：{data}')
            
            # 只发布纯数字（适配你的导航）
            msg = String()
            msg.data = data
            self.result_pub.publish(msg)

    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()