import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from openai import OpenAI
import threading
import numpy as np
import time
import base64
import sys

class AIVisionPublisher(Node):
    def __init__(self):
        super().__init__('ai_vision_publisher')
        self.get_logger().info("正在初始化 AI 视觉发布节点...")

        # 1. 发布者与订阅者
        self.publisher_ = self.create_publisher(String, 'tts_text', 10)
        self.subscription = self.create_subscription(
            CompressedImage, 'image', self.image_callback, 10
        )
        
        self.bridge = CvBridge()
        self.processing_lock = threading.Lock()
        self.processing_active = False

        # 2. 初始化大模型客户端
        self.configure_cloud_services()
        self.get_logger().info("AI 视觉发布节点已就绪，等待图像输入...")

    def configure_cloud_services(self):
        try:
            self.llm_client = OpenAI(
                api_key="sk-ws-H.EPMMRPX.ha41.MEUCIBtDZ00VSRzdWvNtcG0quENn5ZCMCVHFz9GBRq8a_hrDAiEAwVnXtlIFG3kFjVd1tpARpORxGHX59J_j01WBSHTxvBc", 
                base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
            )
            self.get_logger().info("大模型客户端初始化成功。")
        except Exception as e:
            self.get_logger().error(f"大模型客户端初始化失败: {e}")
            sys.exit(1) # 关键：如果初始化失败，直接退出并报错

    def image_callback(self, msg):
        # 频率控制：如果正在处理，直接跳过
        if self.processing_active:
            return
        
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
            
            self.trigger_pipeline(frame)
        except Exception as e:
            self.get_logger().error(f"图像回调处理异常: {str(e)}")

    def trigger_pipeline(self, frame):
        with self.processing_lock:
            if self.processing_active:
                return
            self.processing_active = True
        
        # 使用线程异步处理，避免阻塞图像订阅回调
        threading.Thread(target=self.process_and_publish, args=(frame.copy(),), daemon=True).start()

    def process_and_publish(self, frame):
        try:
            # 编码
            _, buffer = cv2.imencode('.jpg', frame)
            base64_image = base64.b64encode(buffer).decode('utf-8')
            
            # 推理
            description = self.get_image_description_base64(base64_image)
            
            # 发布
            if description and description not in ["描述生成失败", "描述提取异常"]:
                msg = String()
                msg.data = description
                self.publisher_.publish(msg)
                self.get_logger().info(f"发布成功: {msg.data}")

        except Exception as e:
            self.get_logger().error(f"处理流水线崩溃: {str(e)}")
        finally:
            self.processing_active = False

    def get_image_description_base64(self, base64_str):
        try:
            response = self.llm_client.chat.completions.create(
                model="qwen3.7-plus",
                messages=[{
                    "role": "user", 
                    "content": [
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_str}"}},
                        {"type": "text", "text": "请分析这个画面，只分析漫画人物，在30字以内描述，包括人物的性别，年龄，以及所在场景。"}
                    ]
                }],
                max_tokens=50,
                # 关闭思考模式：qwen3.7-plus 默认思考（先生成大量推理内容再回答），
                # 图生文只需直出 30 字描述，关闭后推理速度快很多。
                # 注意：enable_thinking 不是 OpenAI 标准参数，必须走 extra_body 才生效
                # （直接放顶层会被忽略，参考 DashScope 深度思考文档）。
                extra_body={"enable_thinking": False},
            )
            return response.choices[0].message.content
        except Exception as e:
            self.get_logger().warn(f"大模型 API 调用失败: {e}")
            return "描述生成失败"

def main(args=None):
    rclpy.init(args=args)
    node = AIVisionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点停止运行")
    except Exception as e:
        node.get_logger().error(f"节点发生未捕获异常: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()