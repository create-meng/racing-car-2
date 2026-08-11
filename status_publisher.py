import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        # 创建一个发布者，话题名为 'patient_status'
        self.publisher_ = self.create_publisher(String, 'patient_status', 4)
        # 设置定时器，每 10 秒发布一次
        self.timer = self.create_timer(10.0, self.timer_callback)
        self.get_logger().info('发布者节点已启动，正在发布病人状态...')

    def timer_callback(self):
        msg = String()
        msg.data = '一个病人正在躺在床上然后穿着红色衣服，打着点滴'
        self.publisher_.publish(msg)
        self.get_logger().info(f'已发布内容: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = StatusPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()