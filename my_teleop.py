import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
import websockets
import json
import threading
import time

class WebAckermanTeleop(Node):
    def __init__(self):
        super().__init__('web_ackerman_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 速度极限定义
        self.SPEED_MIN = 0.0
        self.SPEED_MAX = 0.8
        self.SPEED_STEP = 0.01  

        # 基础运行线速度
        self.target_linear_abs = 0.3 
        self.base_linear = 0.0

        # 转向参数
        self.STEER_ADD_LINEAR = 0.001  # 转向时提供的独立附加线速度
        self.angular = 0.0
        self.angular_acc = 0.0
        self.ANG_STEP = 0.05
        self.ANG_MAX = 1.0
        
        # 当前网页端按下的键集
        self.pressed_keys = set()

        # 启动后台控制循环线程
        self.loop_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.loop_thread.start()

    def control_loop(self):
        rate = 0.05  # 20Hz
        twist = Twist()
        
        print("==== 阿克曼网页端控制后端已启动 ====")
        print("强制修复：倒车(S)时角速度自动取反，适配特定底盘固件需求。")
        
        while rclpy.ok():
            keys = self.pressed_keys

            # 1. 处理 Shift 加速 与 K 减速
            if 'shift' in keys:
                self.target_linear_abs = min(self.target_linear_abs + self.SPEED_STEP, self.SPEED_MAX)
            if 'k' in keys:
                self.target_linear_abs = max(self.target_linear_abs - self.SPEED_STEP, self.SPEED_MIN)

            # 2. 根据 W/S 决定方向
            if 'w' in keys:
                self.base_linear = self.target_linear_abs
            elif 's' in keys:
                self.base_linear = -self.target_linear_abs
            else:
                self.base_linear = 0.0  

            # 3. 处理停车 P
            if 'p' in keys:
                self.base_linear = 0.0
                self.target_linear_abs = 0.3 
                self.angular_acc = 0.0
                self.angular = 0.0

            # 4. A/D 转向角度累积
            if 'a' in keys:
                self.angular_acc = min(self.angular_acc + self.ANG_STEP, self.ANG_MAX)
                self.angular = self.angular_acc
            elif 'd' in keys:
                self.angular_acc = min(self.angular_acc + self.ANG_STEP, self.ANG_MAX)
                self.angular = -self.angular_acc
            else:
                self.angular_acc = max(self.angular_acc - 0.05, 0.0)
                self.angular = 0.0

            # 5. 最终线速度与角速度的方向适配计算
            final_linear = self.base_linear
            final_angular = self.angular
            
            if self.angular != 0:
                if 's' in keys:
                    # 如果在按住 S 倒车
                    final_linear += (self.STEER_ADD_LINEAR * -1.0)
                    final_angular = self.angular * -1.0  # 核心修复：在这里将倒车角速度强行反转
                else:
                    # 如果是前进（按W）或者没按键原地按A/D
                    final_linear += (self.STEER_ADD_LINEAR * 1.0)
                    final_angular = self.angular

            # 6. 发布速度
            twist.linear.x = final_linear
            twist.angular.z = final_angular  
            self.pub.publish(twist)

            if final_linear != 0 or final_angular != 0:
                print(f"基准速度: {self.target_linear_abs:.2f} | 最终线速度: {twist.linear.x:.3f} | 最终角速度: {twist.angular.z:.2f}", end='\r')

            time.sleep(rate)

    async def ws_handler(self, websocket):
        async for message in websocket:
            try:
                data = json.loads(message)
                event_type = data.get("type")
                key = data.get("key", "").lower()

                if event_type == "keydown":
                    self.pressed_keys.add(key)
                elif event_type == "keyup":
                    self.pressed_keys.discard(key)
            except Exception as e:
                self.get_logger().error(f"解析数据失败: {e}")

async def main_async():
    rclpy.init()
    node = WebAckermanTeleop()

    async with websockets.serve(node.ws_handler, "0.0.0.0", 8765):
        print("WebSocket 服务器已在 0.0.0.0:8765 启动")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            await asyncio.sleep(0.01)
            
    node.destroy_node()
    rclpy.shutdown()

def main():
    asyncio.run(main_async())

if __name__ == '__main__':
    main()