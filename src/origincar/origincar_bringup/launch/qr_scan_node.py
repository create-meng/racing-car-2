import cv2
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class QRDecoderNode(Node):
    def __init__(self):
        super().__init__('qr_decoder_node')
        
        # 1. 初始化 ROS2 发布者 (用于将二维码内容传给其他节点)
        self.publisher_ = self.create_publisher(String, '/qr_code_content', 10)
        
        # 2. 微信二维码模型路径
        model_dir = "/userdata/dev_ws/src/origincar/wechat_qr_models/"
        detect_proto = os.path.join(model_dir, "detect.prototxt")
        detect_model = os.path.join(model_dir, "detect.caffemodel")
        sr_proto = os.path.join(model_dir, "sr.prototxt")
        sr_model = os.path.join(model_dir, "sr.caffemodel")

        # 3. 实例化微信二维码识别器
        try:
            # 检查文件是否存在
            if not os.path.exists(detect_proto):
                raise FileNotFoundError(f"找不到模型文件: {detect_proto}")
            
            self.detector = cv2.wechat_qrcode_WeChatQRCode(
                detect_proto, detect_model, sr_proto, sr_model
            )
            self.get_logger().info("微信二维码识别模型加载成功")
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {str(e)}")
            return

        # 4. 配置摄像头 (最高画质 1920x1280)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1280)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            return

        self.get_logger().info("开始检测... (1920x1280)")
        self.run_detection()

    def run_detection(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                continue

            # 微信扫码核心解码逻辑
            # res 是一个包含所有识别到的二维码内容的字符串列表
            res, _ = self.detector.detectAndDecode(frame)

            for content in res:
                if content:
                    # 1. 打印日志 (终端显示)
                    self.get_logger().info(f"扫码成功: {content}")
                    
                    # 2. 发布到话题
                    msg = String()
                    msg.data = content
                    self.publisher_.publish(msg)

        # 释放资源
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = QRDecoderNode()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()