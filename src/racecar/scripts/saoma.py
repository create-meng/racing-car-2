import cv2
import os
import rclpy
import time  # 用于读帧失败时短暂停顿，避免死循环
from rclpy.node import Node
from std_msgs.msg import String

class QRDecoderNode(Node):
    def __init__(self):
        super().__init__('qr_decoder_node')
        
        # 1. 初始化 ROS2 发布者 (用于将二维码内容传给其他节点)
        self.publisher_ = self.create_publisher(String, '/qr_result', 10)
        
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

        # 4. 配置摄像头
        # 优先使用 V4L2 后端（初始化快、稳定），避免默认 GStreamer 后端慢/报错
        self.cap = self.open_camera()

        # 先尝试最高画质 1920x1280，若摄像头不支持则自动降级到 640x480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1280)
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if w < 1920 or h < 1280:
            self.get_logger().warn(f"摄像头实际分辨率 {w}x{h}，降级到 640x480")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # FOURCC 设置失败不影响启动（GStreamer 后端不支持时仅告警）
        try:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        except Exception as e:
            self.get_logger().warn(f"FOURCC 设置失败（可忽略）: {e}")

        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            self.get_logger().error("请检查摄像头连接，或运行 v4l2-ctl --list-devices 查看可用设备")
            return

        self.get_logger().info(f"开始检测... ({w}x{h})")
        self.run_detection()

    def open_camera(self):
        """打开摄像头：优先 V4L2 后端，重试多次；失败则回退默认后端（GStreamer）"""
        # 1) 等待摄像头设备被释放
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

    def run_detection(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("无法读取摄像头画面，稍后重试...", throttle_duration_sec=2.0)
                time.sleep(0.1)
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