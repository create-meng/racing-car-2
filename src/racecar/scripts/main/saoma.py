import cv2
import glob
import os
import rclpy
import re
import subprocess
import time  # 用于读帧失败时短暂停顿，避免死循环
from rclpy.node import Node
from std_msgs.msg import String

class QRDecoderNode(Node):
    def __init__(self):
        super().__init__('qr_decoder_node')

        self.declare_parameter('camera_index', -1)
        self.camera_index = self.get_parameter('camera_index').value
        
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
        # 优先使用 V4L2 后端（初始化快、稳定），避免默认 GStreamer 后端慢/报错
        # 注意：杀死 pub_image.py 后，摄像头设备需要时间释放，因此等待并多次重试
        self.cap = self.open_camera()

        # 先尝试最高画质 1920x1080，若摄像头不支持则自动降级到 640x480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if w < 1920 or h < 1080:
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

    def find_camera_index(self):
        candidates = sorted(glob.glob('/dev/video*'))
        if not candidates:
            self.get_logger().error("未找到任何 /dev/video* 摄像头设备")
            return None
        for dev in candidates:
            try:
                idx = int(dev.split('video')[-1])
            except ValueError:
                continue
            self.get_logger().info(f"尝试探测摄像头 {dev} (index={idx})...")
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if cap is not None and cap.isOpened():
                ret, frame = cap.read()
                cap.release()
                if ret and frame is not None:
                    self.get_logger().info(f"自动选择摄像头 {dev} (index={idx})")
                    return idx
            self.get_logger().warn(f"{dev} 无法抓取帧，跳过")
        return None

    def open_camera(self):
        """打开摄像头：优先 V4L2 后端，重试多次；失败则回退默认后端（GStreamer）"""
        # 1) 等待旧进程释放摄像头设备（pub_image.py 刚被杀死）
        time.sleep(1.0)

        # 2) 自动探测摄像头索引（camera_index=-1 时）
        if self.camera_index < 0:
            detected = self.find_camera_index()
            if detected is None:
                self.get_logger().error("未探测到可用摄像头")
                return cv2.VideoCapture()
            self.camera_index = detected

        # 3) 优先 V4L2 后端，最多重试 3 次
        for attempt in range(1, 4):
            cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
            if cap is not None and cap.isOpened():
                self.get_logger().info(f"已通过 V4L2 后端打开摄像头 index={self.camera_index}（第 {attempt} 次尝试）")
                return cap
            if cap is not None:
                cap.release()
            self.get_logger().warn(f"V4L2 后端打开失败（第 {attempt}/3 次），等待 1 秒后重试...")
            time.sleep(1.0)

        # 4) V4L2 全部失败，回退默认后端（GStreamer）
        self.get_logger().warn("V4L2 后端多次尝试失败，改用默认后端（GStreamer）...")
        return cv2.VideoCapture(self.camera_index)

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
                self.get_logger().warn("无法读取摄像头画面，稍后重试...", throttle_duration_sec=2.0)
                time.sleep(0.1)
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
                ["python3", script_path, "--ros-args", "-p", f"camera_index:={self.camera_index}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                close_fds=True
            )
            self.get_logger().info(f"已启动: {script_path} (camera_index={self.camera_index})")
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