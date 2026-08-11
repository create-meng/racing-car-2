import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np

class QRCodeCameraNode(Node):
    def __init__(self):
        super().__init__('qr_camera_detector_node')
        
        # 创建二维码结果发布器
        self.result_pub = self.create_publisher(String, '/qr_result', 10)
        
        # 初始化OpenCV摄像头（0=默认摄像头）
        self.cap = cv2.VideoCapture(0)
        
        # 初始化二维码检测器
        self.qr_detector = cv2.QRCodeDetector()
        
        # 创建定时器：每隔30ms读取一帧摄像头画面
        self.timer = self.create_timer(0.03, self.timer_callback)
        
        self.get_logger().info('摄像头二维码检测节点已启动！')
        self.get_logger().info(' 已打开本地摄像头')
        self.get_logger().info(' 结果将发布到话题：/qr_result')

    def timer_callback(self):
        # 读取一帧摄像头图像
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('  无法读取摄像头画面')
            return

        # 解码二维码
        data, bbox, _ = self.qr_detector.detectAndDecode(frame)

        # 如果识别到二维码内容
        if data:
            self.get_logger().info(f' 识别二维码内容：{data}')
            
            # 判断是否为纯数字
            if data.isdigit():
                num = int(data)
                # 判断奇数/偶数
                if num % 2 == 0:
                    result = f"数字 {num} 是 偶数"
                else:
                    result = f"数字 {num} 是 奇数"
            else:
                result = f"内容 {data} 不是纯数字"
            
            # 发布判断结果
            msg = String()
            msg.data = result
            self.result_pub.publish(msg)
            self.get_logger().info(f' 发布结果：{result}')



    def __del__(self):
        # 节点关闭时释放摄像头
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(' 节点已关闭')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()