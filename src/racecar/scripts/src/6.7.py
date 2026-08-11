#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import csv
import re

# ==========================================
# 辅助节点：专职负责监听二维码和发布禁区标记
# ==========================================
class TaskHelperNode(Node):
    def __init__(self):
        super().__init__('nav_task_helper_node')
        self.qr_result = None
        
        # 订阅二维码结果
        self.qr_sub = self.create_subscription(String, "/qr_result", self.qr_callback, 10)
        
        # 发布禁区可视化标记
        self.marker_pub = self.create_publisher(Marker, "/forbidden_zone_marker", 10)
        self.timer = self.create_timer(1.0, self.show_forbidden_zone)

    def qr_callback(self, msg):
        if self.qr_result is not None:
            return  # 已经识别过则忽略
            
        text = msg.data
        self.get_logger().info(f"【收到二维码数据】: {text}")
        nums = re.findall(r'\d+', text)
        if nums:
            self.qr_result = int(nums[0])
            self.get_logger().warn(f"【触发中断】成功提取数字: {self.qr_result}")

    def show_forbidden_zone(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "forbidden_zone"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = (2.35 + 4.3) / 2
        marker.pose.position.y = (1.9 + 2.4) / 2
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 4.3 - 2.35
        marker.scale.y = 2.4 - 1.9
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.3
        self.marker_pub.publish(marker)

# ==========================================
# 全局辅助函数：读取 CSV 并过滤禁区点
# ==========================================
def is_in_forbidden_zone(x, y):
    return 2.35 <= x <= 4.3 and 1.9 <= y <= 2.4

def get_poses_from_csv(file_path, navigator):
    poses = []
    try:
        with open(file_path, 'r') as f:
            reader = csv.reader(f)
            for i, row in enumerate(reader):
                if len(row) == 4:
                    x, y, z, w = map(float, row)
                    
                    # 禁区检查
                    if is_in_forbidden_zone(x, y):
                        navigator.warn(f"跳过危险路点 {i}: ({x:.2f}, {y:.2f}) 在禁区内！")
                        continue
                        
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = navigator.get_clock().now().to_msg()
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.z = z
                    pose.pose.orientation.w = w
                    poses.append(pose)
    except Exception as e:
        navigator.error(f"读取 CSV 文件失败: {file_path}, 错误: {e}")
    
    return poses

# ==========================================
# 主流程控制
# ==========================================
def main():
    rclpy.init()
    
    # 实例化 Nav2 指挥官和我们的后台辅助节点
    navigator = BasicNavigator()
    helper = TaskHelperNode()
    
    navigator.info("等待 Nav2 系统上线...")
    navigator.waitUntilNav2Active()

    # -----------------------
    # 第一阶段：执行 dating.csv
    # -----------------------
    navigator.info("=== 开始第一阶段：dating.csv ===")
    phase1_poses = get_poses_from_csv("/root/ros2_ws/src/racecar/scripts/dating.csv", navigator)
    
    if phase1_poses:
        # 使用 followWaypoints（到点停车），比 NavigateThroughPoses 更稳
        navigator.followWaypoints(phase1_poses)
        
        # 轮询循环：一边检查导航进度，一边监听二维码
        while not navigator.isTaskComplete():
            # 让辅助节点处理一下回调（监听话题）
            rclpy.spin_once(helper, timeout_sec=0.1)
            
            # 如果在导航途中收到了二维码数字，立刻取消当前导航！
            if helper.qr_result is not None:
                navigator.warn("检测到二维码信号，正在强制终止第一阶段任务！")
                navigator.cancelTask()
                break

    # -----------------------
    # 第二阶段：等待二维码并判断
    # -----------------------
    navigator.info("等待二维码识别结果以决定下一步路线...")
    while helper.qr_result is None:
        rclpy.spin_once(helper, timeout_sec=0.1)
        
    num = helper.qr_result
    navigator.info(f"=== 开始第二阶段：基于二维码数字 {num} ===")
    
    if num % 2 == 1:
        navigator.info("判断为奇数：加载 shunshizhen1.csv")
        phase2_file = "/root/ros2_ws/src/racecar/scripts/shunshizhen1.csv"
    else:
        navigator.info("判断为偶数：加载 nishizhen.csv")
        phase2_file = "/root/ros2_ws/src/racecar/scripts/nishizhen/nishizhen.csv"

    phase2_poses = get_poses_from_csv(phase2_file, navigator)

    if phase2_poses:
        navigator.followWaypoints(phase2_poses)
        
        while not navigator.isTaskComplete():
            rclpy.spin_once(helper, timeout_sec=0.1)
            
        # 获取最终结果
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.info('目标达成，第二阶段完美跑完全程！')
        elif result == TaskResult.CANCELED:
            navigator.warn('第二阶段任务被手动取消。')
        elif result == TaskResult.FAILED:
            navigator.error('第二阶段任务失败。')
    else:
        navigator.error("第二阶段未能加载任何有效路点，任务结束。")

    # 收尾工作
    helper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()