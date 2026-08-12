import cv2
import os
import rclpy
import re
import subprocess
from rclpy.node import Node
from std_msgs.msg import String

class QRDecoderNode(Node):
    def __init__(self):
        super().__init__('qr_decoder_node')
        
        self.publisher_ = self.create_publisher(String, '/qr_result', 10)
        self.tts_publisher_ = self.create_publisher(String, '/tts_text', 10)
        
        # 先杀死已存在的图像发布节点
        self.kill_pub_image()

        model_dir = "/userdata/dev_ws/src/origincar/wechat_qr_models/"
        detect_proto = os.path.join(model_dir, "detect.prototxt")
        detect_model = os.path.join(model_dir, "detect.caffemodel")
        sr_proto = os.path.join(model_dir, "sr.prototxt")
        sr_model = os.path.join(model_dir, "sr.caffemodel")

        try:
            if not os.path.exists(detect_proto):
                raise FileNotFoundError(f"找不到模型文件: {detect_proto}")
            
            self.detector = cv2.wechat_qrcode_WeChatQRCode(
                detect_proto, detect_model, sr_proto, sr_model
            )
            self.get_logger().info("微信二维码识别模型加载成功")
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {str(e)}")
            return

        # 杀死旧进程后再打开摄像头
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            return

        self.get_logger().info("开始检测... (1920x1080)")
        self.run_detection()

    def kill_pub_image(self):
        """在启动摄像头前杀死 pub_image.py 进程"""
        script_name = "pub_image.py"
        try:
            # pkill -f 精确匹配进程名
            subprocess.run(
                ["pkill", "-f", script_name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info(f"已尝试杀死进程: {script_name}")
        except Exception as e:
            self.get_logger().warn(f"杀死进程时出现异常(可能已不存在): {e}")

    def run_detection(self):
        qr_detected = False

        while rclpy.ok() and not qr_detected:
            ret, frame = self.cap.read()
            if not ret:
                continue

            res, _ = self.detector.detectAndDecode(frame)

            for content in res:
                if content:
                    self.get_logger().info(f"扫码成功: {content}")

                    msg = String()
                    msg.data = content
                    self.publisher_.publish(msg)

                    tts_msg = String()
                    digits_only = re.sub(r'\D', '', content)
                    if digits_only:
                        tts_msg.data = digits_only
                        self.get_logger().info(f"发送数字到TTS: {tts_msg.data}")
                    else:
                        tts_msg.data = content
                        self.get_logger().info(f"发送原文到TTS: {tts_msg.data}")

                    self.tts_publisher_.publish(tts_msg)

                    qr_detected = True
                    break

        # 释放摄像头
        self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("摄像头已释放")

        # 重新启动图像发布
        script_path = "/root/ros2_ws/src/racecar/scripts/main/pub_image.py"
        try:
            subprocess.Popen(
                ["python3", script_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                close_fds=True
            )
            self.get_logger().info(f"已启动: {script_path}")
        except Exception as e:
            self.get_logger().error(f"启动脚本失败: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = QRDecoderNode()
    except KeyboardInterrupt:
        print("程序被手动中断")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()