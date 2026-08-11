import cv2
import os
import rclpy
import re  # 新增：导入正则表达式库，用于过滤和提取数字
from rclpy.node import Node
from std_msgs.msg import String

class QRDecoderNode(Node):
    def __init__(self):
        super().__init__('qr_decoder_node')
        
        # 1. 初始化 ROS2 发布者 (用于将二维码内容传给其他节点)
        self.publisher_ = self.create_publisher(String, '/qr_result', 10)
        
        # 【新增】 初始化 TTS 发布者，发布到 /tts_text 话题用于语音播报
        self.tts_publisher_ = self.create_publisher(String, '/tts_text', 10)
        
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
                    
                    # 2. 发布到原有的 /qr_result 话题
                    msg = String()
                    msg.data = content
                    self.publisher_.publish(msg)

                    # 【新增】 3. 发布数字到 /tts_text 话题用于语音播报
                    tts_msg = String()
                    
                    # 提取字符串中的所有数字 (例如 "Box123" 会提取为 "123")
                    # 如果你希望原封不动地发送完整文本，直接改为 tts_msg.data = content 即可
                    digits_only = re.sub(r'\D', '', content) 
                    
                    if digits_only:
                        tts_msg.data = digits_only
                        self.tts_publisher_.publish(tts_msg)
                        self.get_logger().info(f"已发送数字到 TTS 话题: {tts_msg.data}")
                    else:
                        # 如果没有数字，兜底发送原内容
                        tts_msg.data = content
                        self.tts_publisher_.publish(tts_msg)
                        self.get_logger().info(f"未提取到数字，已发送原内容到 TTS 话题: {tts_msg.data}")

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