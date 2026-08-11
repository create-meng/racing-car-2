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
import base64
import subprocess  # 用来替换 os.system 提高稳定性

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
        self.get_logger().info("当前运行模式：内存Base64直传极速优化版")
        self.get_logger().info("等待摄像头节点按 'T' 键发送图像...")

    def configure_cloud_services(self):
        # 优先从环境变量获取密钥，防止代码硬编码泄露
        api_key = os.getenv("ARK_API_KEY", "ark-b217b3ed-f4b8-4b5b-9265-73893bf346be-13833")
        
        # 配置火山引擎大模型客户端
        self.llm_client = OpenAI(
            api_key=api_key, 
            base_url="https://ark.cn-beijing.volces.com/api/v3"
        )

    def image_callback(self, msg):
        # 如果当前正在识别或正在说话，直接丢弃新来的图像，防止声音重叠和死锁
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

        # 开启独立线程运行完整管线，保证主事件循环的高响应率
        threading.Thread(target=self.process_and_speak, args=(frame.copy(),)).start()

    def process_and_speak(self, frame):
        """图像处理、大模型调用、语音播报的一条龙管线"""
        total_start_time = time.time()
        
        try:
            # ================= 1. 内存图像编码 =================
            self.get_logger().info("[步骤 1/2] 正在压缩并执行本地图像 Base64 编码...")
            step1_start = time.time()
            
            # 限制编码质量为 75% 以减小网络 Payload 大小，同时完全保留大模型所需的语义特征
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
            _, buffer = cv2.imencode('.jpg', frame, encode_param)
            base64_image = base64.b64encode(buffer).decode('utf-8')
            
            step1_end = time.time()
            self.get_logger().info(f"[步骤 1/2] 本地编码完成，耗时: {step1_end - step1_start:.4f} 秒")

            # ================= 2. 大模型推理 =================
            self.get_logger().info("[步骤 2/2] 正在发送高并发网络请求至豆包大模型...")
            step2_start = time.time()
            description = self.get_image_description_base64(base64_image)
            step2_end = time.time()
            
            self.get_logger().info(f"[步骤 2/2] 推理完成，大模型耗时: {step2_end - step2_start:.2f} 秒")
            self.get_logger().info(f"AI描述内容: {description}")

            # ================= 3. 语音生成与播报 =================
            if description and description not in ["描述生成失败", "描述提取异常"]:
                self.get_logger().info("正在启动底层语音模组...")
                self.speak_text(description)

        except Exception as e:
            self.get_logger().error(f"处理管线发生崩溃: {str(e)}")
            
        finally:
            total_end_time = time.time()
            self.get_logger().info(f"端到端总计处理耗时: {total_end_time - total_start_time:.2f} 秒")
            
            # 流程结束，安全释放互斥锁
            with self.processing_lock:
                self.processing_active = False
            self.get_logger().info("就绪，等待下一次按键触发 \n" + "-"*40)

    def get_image_description_base64(self, base64_str):
        try:
            response = self.llm_client.responses.create(
                model="doubao-seed-2-0-mini-260428",
                input=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "input_image",
                                "image_url": f"data:image/jpeg;base64,{base64_str}"
                            },
                            {
                                "type": "input_text",
                                "text": "请分析这个画面，只分析漫画人物，在30字以内描述，包括人物的性别，年龄，以及所在场景。"
                            }
                        ]
                    }
                ]
                # 已移除不支持的 max_tokens 参数
            )
            
            res_dict = response.model_dump() if hasattr(response, 'model_dump') else vars(response)
            description = "描述提取异常"
            
            if 'output' in res_dict and isinstance(res_dict['output'], list):
                for item in res_dict['output']:
                    if item.get('type') == 'message' and 'content' in item:
                        for sub_content in item['content']:
                            if sub_content.get('type') == 'output_text':
                                description = sub_content.get('text', description)
                                break
            
            return description[:50] if len(description) > 50 else description

        except Exception as e:
            self.get_logger().error(f"大模型调用请求失败: {str(e)}")
            return "描述生成失败"

    def speak_text(self, text_content):
        audio_filename = "ai_direct_tts.mp3"
        try:
            # 1. 语音合成（网络请求）
            gen_start = time.time()
            tts_command = f'/usr/local/bin/edge-tts --text "{text_content}" --voice zh-CN-XiaoxiaoNeural --write-media {audio_filename}'
            subprocess.run(tts_command, shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            gen_end = time.time()
            self.get_logger().info(f"  -> 网络语音合成耗时: {gen_end - gen_start:.2f} 秒")
            
            # 2. 扬声器物理播放
            play_start = time.time()
            play_command = f"/usr/bin/mplayer -nolirc -vo null -ao alsa:device=hw=1.0 -really-quiet {audio_filename}"
            # 设置 15 秒强制超时，防止硬件死锁导致程序线程永久挂起
            subprocess.run(play_command, shell=True, check=True, timeout=15)
            play_end = time.time()
            self.get_logger().info(f"  -> 扬声器物理播放耗时: {play_end - play_start:.2f} 秒")
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('语音播放超时，强制终止音频进程。')
        except Exception as e:
            self.get_logger().error(f'语音模块执行失败: {e}')
        finally:
            # 稳健性清理：无论是否成功播放，均检查并移除本地音频碎屑
            if os.path.exists(audio_filename):
                try:
                    os.remove(audio_filename)
                except Exception:
                    pass

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