import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os

class WeChatQRNode(Node):
    def __init__(self):
        super().__init__('wechat_qr_node')
        
        # 1. 声明参数
        self.declare_parameter('model_path', '/userdata/dev_ws/src/origincar/wechat_qr_models/')
        self.declare_parameter('image_topic', '/image')
        self.declare_parameter('pub_topic', '/qr_code_result')

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value

        # 2. 初始化微信扫码引擎
        # 需要检测模型和超分辨率模型共4个文件
        detect_pro = os.path.join(model_path, "detect.prototxt")
        detect_res = os.path.join(model_path, "detect.caffemodel")
        sr_pro = os.path.join(model_path, "sr.prototxt")
        sr_res = os.path.join(model_path, "sr.caffemodel")

        try:
            self.detector = cv2.wechat_qrcode_WeChatQRCode(
                detect_pro, detect_res, sr_pro, sr_res
            )
            self.get_logger().info("微信扫码模型加载成功！")
        except Exception as e:
            self.get_logger().error(f"模型加载失败，请检查路径: {e}")

        # 3. 创建订阅者、发布者和转换工具
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(String, pub_topic, 10)
        
        self.get_logger().info(f"正在订阅 {image_topic} 并发布识别结果至 {pub_topic}")

    def listener_callback(self, msg):
        try:
            # 将 ROS Image 消息转换为 OpenCV 图像
            # 因为原始代码中 /image 是经过 jpeg_codec 发布的，通常是 bgr8 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 执行识别 (res 为识别到的字符串列表)
            res, points = self.detector.detectAndDecode(cv_image)

            if res:
                for content in res:
                    if content: # 确保内容不为空
                        result_msg = String()
                        result_msg.data = content
                        self.publisher_.publish(result_msg)
                        self.get_logger().info(f"扫描到二维码: {content}")
                        
        except Exception as e:
            self.get_logger().error(f"处理图像失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WeChatQRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()