#!/usr/bin/env python3
"""
二维码识别节点（USB + 深度相机双路并行，谁快用谁）

- USB 摄像头：独立线程循环读帧识别（V4L2，优先 1920x1080，降级 640x480）
- 深度相机（Aurora930）：订阅 /aurora/rgb/image_raw（BGR888），回调中识别
- 两路同时运行，谁先识别到二维码谁先发布 /qr_result（h1.py 只取第一个结果）
- 识别成功后：发布 /qr_result 与 /tts_text（与旧版行为一致），释放 USB 摄像头，
  确保图生文图像节点（pub_image.py）已启动，随后自动退出（仅识别一次）
"""
import cv2
import glob
import os
import re
import rclpy
import subprocess
import threading
import time  # 用于读帧失败时短暂停顿，避免死循环
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class QRDecoderNode(Node):
    def __init__(self):
        super().__init__('qr_decoder_node')
        self.init_ok = False

        self.declare_parameter('camera_index', -1)
        self.camera_index = self.get_parameter('camera_index').value

        self.publisher_ = self.create_publisher(String, '/qr_result', 10)
        self.tts_publisher_ = self.create_publisher(String, '/tts_text', 10)

        # 先杀死已存在的图像发布节点（旧版 pub_image.py 会占用 USB 摄像头）
        self.kill_pub_image()

        # 加载微信二维码模型（USB 与深度各一个实例，两路并行识别互不干扰）
        try:
            self.usb_detector = self.load_detector()
            self.depth_detector = self.load_detector()
            self.get_logger().info("微信二维码识别模型加载成功（USB + 深度）")
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

        self.bridge = CvBridge()
        self.done = False            # 已识别成功，停止两路识别
        self.done_lock = threading.Lock()
        self.last_depth_proc_t = 0.0 # 深度识别节流时间戳（约 10Hz）

        # 订阅深度相机 RGB 流（Aurora930 驱动节点，namespace=aurora）
        # 使用传感器 QoS（best effort），兼容驱动发布端的 QoS
        self.image_sub = self.create_subscription(
            Image,
            '/aurora/rgb/image_raw',
            self.depth_image_callback,
            qos_profile=qos_profile_sensor_data,
        )

        # 提前确保图生文图像节点(pub_image.py)已运行：
        # 上次运行教训——扫码成功后才启动 pub_image 太晚，会错过 h1.py 发布的
        # /special_goal_topic 触发消息（volatile 不重放），导致图生文截不到图。
        # 现在扫码阶段就让它在线，h1.py 触发时必然能收到。
        self.ensure_pub_image_running()

        # 同时确保大模型图生文节点(ai_qw.py)已运行：
        # 上次运行教训——截图和 image 话题都发布成功了，但 ai_qw.py 没启动，
        # 图像没人消费，大模型不生成描述、没有语音播报，看起来"图生文没触发"。
        # 现在扫码阶段一起拉起，触发时整条链路（截深度图 → 大模型生文 → TTS）必然就绪。
        self.ensure_ai_qw_running()

        # USB 摄像头识别放独立线程，主线程循环 spin_once 处理深度相机订阅
        self.usb_thread = threading.Thread(target=self.usb_detect_loop, daemon=True)
        self.usb_thread.start()

        self.get_logger().info(f"开始检测... ({w}x{h}) USB 摄像头 + 深度相机 双路并行")
        self.init_ok = True

    def load_detector(self):
        """加载微信二维码识别模型（USB 与深度各一份）。"""
        model_dir = "/userdata/dev_ws/src/origincar/wechat_qr_models/"
        detect_proto = os.path.join(model_dir, "detect.prototxt")
        detect_model = os.path.join(model_dir, "detect.caffemodel")
        sr_proto = os.path.join(model_dir, "sr.prototxt")
        sr_model = os.path.join(model_dir, "sr.caffemodel")
        if not os.path.exists(detect_proto):
            raise FileNotFoundError(f"找不到模型文件: {detect_proto}")
        return cv2.wechat_qrcode_WeChatQRCode(
            detect_proto, detect_model, sr_proto, sr_model
        )

    def usb_detect_loop(self):
        """USB 摄像头识别线程。"""
        while rclpy.ok() and not self.done:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("无法读取摄像头画面，稍后重试...", throttle_duration_sec=2.0)
                time.sleep(0.1)
                continue

            try:
                res, _ = self.usb_detector.detectAndDecode(frame)
            except Exception as e:
                self.get_logger().warn(f"USB 二维码识别异常: {e}", throttle_duration_sec=2.0)
                continue

            for content in res:
                if content:
                    self.on_detected(content, "USB摄像头")
                    break

    def depth_image_callback(self, msg):
        """深度相机 RGB 帧回调：与 USB 路并行识别，谁快用谁。"""
        if self.done:
            return

        # 节流：限制识别频率约 10Hz，避免 CPU 占用过高
        now = time.time()
        if now - self.last_depth_proc_t < 0.1:
            return
        self.last_depth_proc_t = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        try:
            res, _ = self.depth_detector.detectAndDecode(frame)
        except Exception as e:
            self.get_logger().warn(f"深度二维码识别异常: {e}", throttle_duration_sec=2.0)
            return

        for content in res:
            if content:
                self.on_detected(content, "深度相机")
                break

    def on_detected(self, content, source):
        """两路共用：识别成功 → 发布结果与TTS → 标记完成（仅一次）。"""
        with self.done_lock:
            if self.done:  # 另一路已识别成功，忽略
                return
            self.done = True

        self.get_logger().info(f"【{source}】扫码成功: {content}")

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
        """在启动摄像头前杀死 pub_image.py 进程（旧版会占用 USB 摄像头）"""
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

    def ensure_pub_image_running(self):
        """确保 pub_image.py（图生文图像源）在运行：已存在则跳过，避免重复进程。"""
        try:
            ret = subprocess.run(
                ["pgrep", "-f", "pub_image.py"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            if ret.returncode == 0:
                self.get_logger().info("pub_image.py 已在运行，跳过启动")
                return
        except Exception:
            pass

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

    def ensure_ai_qw_running(self):
        """确保 ai_qw.py（大模型图生文节点）在运行：已存在则跳过，避免重复进程。"""
        try:
            ret = subprocess.run(
                ["pgrep", "-f", "ai_qw.py"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            if ret.returncode == 0:
                self.get_logger().info("ai_qw.py 已在运行，跳过启动")
                return
        except Exception:
            pass

        script_path = "/root/ros2_ws/src/racecar/scripts/main/ai_qw.py"
        try:
            subprocess.Popen(
                ["python3", script_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                close_fds=True,
            )
            self.get_logger().info(f"已启动: {script_path}（大模型图生文节点）")
        except Exception as e:
            self.get_logger().error(f"启动脚本失败: {str(e)}")

    def cleanup(self):
        """识别结束后的收尾：等待USB线程退出、释放摄像头、确保图生文链路节点已启动。"""
        if getattr(self, 'usb_thread', None) is not None:
            self.usb_thread.join(timeout=1.0)
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
            cv2.destroyAllWindows()
            self.get_logger().info("USB摄像头已释放")
        # 兜底：即使中途节点挂了，也确保扫码结束后图生文链路（截图+大模型）在线
        self.ensure_pub_image_running()
        self.ensure_ai_qw_running()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = QRDecoderNode()
        if node.init_ok:
            # 主循环：spin_once 处理深度相机订阅；识别成功后 done=True，立即退出
            # （上次运行教训：依赖定时器 rclpy.shutdown() 会被深度识别回调饿死，
            #   导致 39 秒后才退出、pub_image 启动太晚错过图生文触发）
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        print("程序被手动中断")
    finally:
        if node:
            node.cleanup()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
