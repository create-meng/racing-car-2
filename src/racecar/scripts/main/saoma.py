import cv2
import glob
import os
import rclpy
import re
import time
import threading
import base64
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from openai import OpenAI


class MergedQRNode(Node):
    """合并版：双摄像头扫码 + 图生文 + image话题发布"""

    def __init__(self):
        super().__init__('merged_qr_node')

        self.declare_parameter('camera_index', -1)
        self.camera_index = self.get_parameter('camera_index').value

        # ===== 发布者 =====
        self.qr_pub = self.create_publisher(String, '/qr_result', 10)
        self.tts_pub = self.create_publisher(String, '/tts_text', 10)
        self.image_pub = self.create_publisher(CompressedImage, 'image', 10)

        # ===== 订阅者 =====
        self.create_subscription(Image, '/aurora/rgb/image_raw', self.depth_image_callback, 10)
        self.create_subscription(String, '/special_goal_topic', self.special_goal_callback, 10)

        self.bridge = CvBridge()
        self.qr_detected = False
        self.latest_depth_frame = None
        self.last_qr_content = ""
        self.scan_lock = threading.Lock()

        # ===== 加载微信二维码模型 =====
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

        # ===== 打开USB摄像头（V4L2） =====
        self.usb_cap = self.open_usb_camera()
        if not self.usb_cap or not self.usb_cap.isOpened():
            self.get_logger().warn("USB摄像头打开失败，仅使用深度相机扫码")
        else:
            w = int(self.usb_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self.usb_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.get_logger().info(f"USB摄像头就绪: {w}x{h}")

        # ===== 初始化AI大模型客户端 =====
        self.init_ai_client()
        self.ai_processing = False

        # ===== 启动USB扫码线程 =====
        if self.usb_cap and self.usb_cap.isOpened():
            threading.Thread(target=self.usb_qr_loop, daemon=True).start()
            self.get_logger().info("USB摄像头扫码线程已启动")
        self.get_logger().info("深度相机RGB扫码线程已启动（通过话题回调）")

        # ===== 定时器：发布image话题（10fps） =====
        self.create_timer(0.1, self.image_pub_callback)
        self.get_logger().info("image话题发布定时器已启动 (10fps)")

        self.get_logger().info("===== 合并版节点启动完成 =====")
        self.get_logger().info("【扫码】双摄像头同时扫码，先检测到者发布 /qr_result")
        self.get_logger().info("【图生文】收到 /special_goal_topic 后使用深度相机RGB做AI分析")

    # ==============================
    # USB摄像头打开
    # ==============================
    def find_camera_index(self):
        candidates = sorted(glob.glob('/dev/video*'))
        if not candidates:
            self.get_logger().error("未找到任何 /dev/video* 摄像头设备")
            return None

        # 只保留 video 编号为偶数的设备（奇数通常是 metadata 设备，非捕获设备）
        # 例如 /dev/video0 是捕获设备，/dev/video1 是 metadata 设备
        capture_candidates = []
        for dev in candidates:
            try:
                idx = int(dev.split('video')[-1])
                if idx % 2 == 0:  # 偶数索引通常是捕获设备
                    capture_candidates.append((dev, idx))
            except ValueError:
                continue

        if not capture_candidates:
            capture_candidates = [(dev, int(dev.split('video')[-1])) for dev in candidates]

        for dev, idx in capture_candidates:
            # 每个设备重试多次，解决摄像头短暂被占用的问题
            for retry in range(1, 4):
                self.get_logger().info(f"尝试探测摄像头 {dev} (index={idx}, 第{retry}次)...")
                cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                if cap is not None and cap.isOpened():
                    ret, frame = cap.read()
                    cap.release()
                    if ret and frame is not None:
                        self.get_logger().info(f"自动选择摄像头 {dev} (index={idx})")
                        return idx
                    self.get_logger().warn(f"{dev} 能打开但读帧失败（第{retry}次）")
                else:
                    self.get_logger().warn(f"{dev} 无法打开（第{retry}次）")
                if cap is not None:
                    cap.release()
                if retry < 3:
                    time.sleep(1.0)  # 等待后重试
        return None

    def open_usb_camera(self):
        """打开USB摄像头：优先V4L2，重试多次"""
        if self.camera_index < 0:
            detected = self.find_camera_index()
            if detected is None:
                self.get_logger().error("未探测到可用USB摄像头")
                return None
            self.camera_index = detected

        for attempt in range(1, 4):
            cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
            if cap is not None and cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.get_logger().info(f"已通过V4L2打开USB摄像头 index={self.camera_index}（第{attempt}次）")
                return cap
            if cap is not None:
                cap.release()
            self.get_logger().warn(f"V4L2打开失败（第{attempt}/3次），1秒后重试...")
            time.sleep(1.0)

        self.get_logger().warn("V4L2多次尝试失败，改用默认后端...")
        return cv2.VideoCapture(self.camera_index)

    # ==============================
    # 功能1: 双摄像头扫码
    # ==============================
    def usb_qr_loop(self):
        """线程1: USB摄像头连续扫码"""
        while rclpy.ok() and not self.qr_detected:
            try:
                ret, frame = self.usb_cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue

                with self.scan_lock:
                    res, _ = self.detector.detectAndDecode(frame)
                for content in res:
                    if content and content != self.last_qr_content:
                        self.on_qr_detected(content)
                        return
            except Exception as e:
                self.get_logger().warn(f"USB扫码帧处理异常，继续下一帧: {e}")
                time.sleep(0.1)

    def depth_image_callback(self, msg):
        """深度相机RGB帧到达 → 缓存帧 + 扫码"""
        if self.qr_detected:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_depth_frame = frame

            with self.scan_lock:
                res, _ = self.detector.detectAndDecode(frame)
            for content in res:
                if content and content != self.last_qr_content:
                    self.on_qr_detected(content)
                    return
        except Exception as e:
            self.get_logger().warn(f"深度相机帧处理失败: {e}")

    def on_qr_detected(self, content):
        """检测到二维码 → 发布结果，停止扫码"""
        self.qr_detected = True
        self.last_qr_content = content
        self.get_logger().info(f"扫码成功: {content}")

        # 发布 /qr_result
        msg = String()
        msg.data = content
        self.qr_pub.publish(msg)

        # 发布 /tts_text（纯数字）
        digits_only = re.sub(r'\D', '', content)
        tts = String()
        tts.data = digits_only if digits_only else content
        self.tts_pub.publish(tts)
        self.get_logger().info(f"发送TTS: {tts.data}")

        # 释放USB摄像头
        if self.usb_cap and self.usb_cap.isOpened():
            self.usb_cap.release()
            self.get_logger().info("USB摄像头已释放")

    # ==============================
    # 功能2: AI图生文
    # ==============================
    def init_ai_client(self):
        try:
            self.llm_client = OpenAI(
                api_key="sk-ws-H.RPMHDDI.4vTi.MEQCIEEeDfg1s9MR7Q7_UNmOHCQXMODOw00ApoK_8z7-1W_NAiBO2vnra0ag43bUmzG5ChVfruMOTca994eRhIXQm4HQOg",
                base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
            )
            self.get_logger().info("AI大模型客户端初始化成功")
        except Exception as e:
            self.get_logger().error(f"AI大模型客户端初始化失败: {e}")
            self.llm_client = None

    def special_goal_callback(self, msg):
        """收到导航触发 → 用深度相机RGB做AI图生文"""
        if self.latest_depth_frame is None:
            self.get_logger().warn("深度相机尚未就绪，无法做图生文")
            return
        if self.llm_client is None:
            self.get_logger().error("AI客户端未初始化，无法做图生文")
            return
        if self.ai_processing:
            self.get_logger().warn("AI正在处理中，忽略本次触发")
            return

        self.get_logger().info(f"收到触发指令: '{msg.data}'，开始AI图生文...")
        threading.Thread(target=self.ai_analyze, args=(self.latest_depth_frame.copy(),), daemon=True).start()

    def ai_analyze(self, frame):
        """AI图生文：调用大模型描述画面"""
        self.ai_processing = True
        try:
            _, buffer = cv2.imencode('.jpg', frame)
            base64_image = base64.b64encode(buffer).decode('utf-8')

            response = self.llm_client.chat.completions.create(
                model="qwen3-vl-flash",
                messages=[{
                    "role": "user",
                    "content": [
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}},
                        {"type": "text", "text": "请分析这个画面，只分析漫画人物，在30字以内描述，包括人物的性别，年龄，以及所在场景。"}
                    ]
                }],
                max_tokens=50
            )
            description = response.choices[0].message.content
            if description and description not in ["描述生成失败", "描述提取异常"]:
                msg = String()
                msg.data = description
                self.tts_pub.publish(msg)
                self.get_logger().info(f"AI图生文发布成功: {description}")
            else:
                self.get_logger().warn(f"AI图生文结果无效: {description}")
        except Exception as e:
            self.get_logger().error(f"AI图生文失败: {e}")
        finally:
            self.ai_processing = False

    # ==============================
    # 功能3: image话题发布
    # ==============================
    def image_pub_callback(self):
        """定时发布深度相机RGB画面到 image 话题"""
        if self.latest_depth_frame is not None:
            try:
                msg = self.bridge.cv2_to_compressed_imgmsg(self.latest_depth_frame, dst_format='jpg')
                self.image_pub.publish(msg)
            except Exception:
                pass  # 静默失败，不影响主流程


def main(args=None):
    rclpy.init(args=args)
    node = MergedQRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("程序被手动中断")
    finally:
        if hasattr(node, 'usb_cap') and node.usb_cap and node.usb_cap.isOpened():
            node.usb_cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()