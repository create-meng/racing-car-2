import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from openai import OpenAI
import threading
import numpy as np
import os
import time
import base64  # 新增：用于将图像转为 Base64 编码

class AIVisionTTSProcessor(Node):
    def __init__(self):
        super().__init__('ai_vision_tts_processor')

        # 订阅由摄像头节点发来的压缩图像
        self.subscription = self.create_subscription(
            CompressedImage,
            'image',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.configure_cloud_services()

        # 状态控制锁：确保整个流程（识别+播报）完全结束后，才处理下一张图
        self.processing_lock = threading.Lock()
        self.processing_active = False

        self.get_logger().info("AI视觉与语音直出节点已启动！")
        self.get_logger().info("已切换至阿里云 Qwen 视觉大模型 (Base64直传加速版)")
        self.get_logger().info("等待摄像头节点按 'T' 键发送图像...")

    def configure_cloud_services(self):
        # 配置阿里云百炼 (DashScope) 大模型客户端
        self.llm_client = OpenAI(
            api_key="sk-ws-H.RPMHDDI.4vTi.MEQCIEEeDfg1s9MR7Q7_UNmOHCQXMODOw00ApoK_8z7-1W_NAiBO2vnra0ag43bUmzG5ChVfruMOTca994eRhIXQm4HQOg", 
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
        )

    def image_callback(self, msg):
        # 如果当前正在识别或正在说话，直接丢弃新来的图像，防止声音重叠
        if self.processing_active:
            self.get_logger().warn("当前正在处理上一帧或正在播报，忽略本次触发。")
            return

        try:
            # 解析图像
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                self.get_logger().warn("图像解析失败")
                return

            # 触发处理流程
            self.trigger_pipeline(frame)

        except Exception as e:
            self.get_logger().error(f"图像回调异常: {str(e)}")

    def trigger_pipeline(self, frame):
        with self.processing_lock:
            if self.processing_active:
                return
            self.processing_active = True

        # 开启独立线程运行完整管线
        threading.Thread(target=self.process_and_speak, args=(frame.copy(),)).start()

    def process_and_speak(self, frame):
        """图像处理、大模型调用、语音播报的一条龙管线"""
        
        # 记录整个流程的开始时间
        total_start_time = time.time()
        
        try:
            # ================= 1. 本地图像编码与大模型推理 =================
            self.get_logger().info("[步骤 1/2] 正在编码图像并调用阿里云大模型...")
            step1_start = time.time()
            
            # 直接在内存中将 OpenCV 图像转为 Base64 字符串，彻底跳过图床上传
            _, buffer = cv2.imencode('.jpg', frame)
            base64_image = base64.b64encode(buffer).decode('utf-8')
            
            # 直接调用大模型
            description = self.get_image_description_base64(base64_image)
            step1_end = time.time()
            
            self.get_logger().info(f"[步骤 1/2] 完成，编码与推理总耗时: {step1_end - step1_start:.2f} 秒")
            self.get_logger().info(f"AI描述: {description}")

            # ================= 2. 语音生成与播报 =================
            if description and description not in ["描述生成失败", "描述提取异常"]:
                self.get_logger().info("[步骤 2/2] 正在生成并播放语音...")
                step2_start = time.time()
                self.speak_text(description)
                step2_end = time.time()
                self.get_logger().info(f"[步骤 2/2] 完成，语音总耗时: {step2_end - step2_start:.2f} 秒")

        except Exception as e:
            self.get_logger().error(f"处理管线发生崩溃: {str(e)}")
            
        finally:
            total_end_time = time.time()
            self.get_logger().info(f"总计处理耗时: {total_end_time - total_start_time:.2f} 秒")
            
            # 释放锁，允许接收下一张图片的触发
            with self.processing_lock:
                self.processing_active = False
            self.get_logger().info("流程结束，就绪等待下一次按键触发 \n" + "-"*40)

    def get_image_description_base64(self, base64_str):
        try:
            response = self.llm_client.chat.completions.create(
                model="qwen3-vl-flash",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image_url",
                                "image_url": {
                                    # 使用标准的 data URI 格式传递 base64 图像
                                    "url": f"data:image/jpeg;base64,{base64_str}"
                                },
                            },
                            {
                                "type": "text", 
                                "text": "请分析这个画面，只分析漫画人物，在30字以内描述，包括人物的性别，年龄，以及所在场景。"
                            },
                        ],
                    },
                ],
                # 限制最大生成 token 数，防止模型过度思考导致耗时增加
                max_tokens=50 
            )
            
            description = response.choices[0].message.content
            
            # 保底限制字符数，防止生成的描述过长导致播报时间太久
            return description[:50] if len(description) > 50 else description

        except Exception as e:
            self.get_logger().error(f"大模型调用请求异常: {str(e)}")
            return "描述生成失败"

    def speak_text(self, text_content):
        try:
            audio_filename = "ai_direct_tts.mp3"
            
            # [子步骤 2.1] 记录语音合成（网络请求）耗时
            gen_start = time.time()
            tts_command = f'/usr/local/bin/edge-tts --text "{text_content}" --voice zh-CN-XiaoxiaoNeural --write-media {audio_filename}'
            os.system(tts_command)
            gen_end = time.time()
            self.get_logger().info(f"  -> [2.1] 网络语音合成耗时: {gen_end - gen_start:.2f} 秒")
            
            # [子步骤 2.2] 记录扬声器物理播放耗时
            play_start = time.time()
            play_command = f"/usr/bin/mplayer -nolirc -vo null -ao alsa:device=hw=1.0 -really-quiet {audio_filename}"
            os.system(play_command)
            play_end = time.time()
            self.get_logger().info(f"  -> [2.2] 扬声器物理播放耗时: {play_end - play_start:.2f} 秒")
            
            # 清理文件
            if os.path.exists(audio_filename):
                os.remove(audio_filename)
                
        except Exception as e:
            self.get_logger().error(f'语音模块执行失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AIVisionTTSProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()