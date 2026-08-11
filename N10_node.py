#!/usr/bin/env python3
"""
N10激光雷达ROS 2驱动节点
功能：直接读取N10雷达串口原始数据，转换为标准的sensor_msgs/msg/LaserScan消息并发布
"""

import serial
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from collections import deque
import struct

class N10LidarDriver(Node):
    def __init__(self):
        super().__init__('n10_lidar_driver')
        
        # 从ROS参数服务器获取配置参数
        self.declare_parameter('port_name', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 230400)
        self.declare_parameter('frame_id', 'laser_link')
        self.declare_parameter('range_min', 0.05)     # 最小距离 0.05m
        self.declare_parameter('range_max', 12.0)     # 最大距离 12.0m
        self.declare_parameter('invert_frame', False) # 是否反转坐标系
        self.declare_parameter('angle_offset', 0.0)   # 角度偏移（弧度）
        
        # 获取参数值
        self.port_name = self.get_parameter('port_name').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.invert_frame = self.get_parameter('invert_frame').value
        self.angle_offset = self.get_parameter('angle_offset').value
        
        # 创建/scan话题发布器
        self.scan_pub = self.create_publisher(
            LaserScan, 
            'scan', 
            10
        )
        
        # 雷达数据存储：存储0-359度每个角度的最新距离（单位：米）
        self.lidar_data_360 = [0.0] * 360
        
        # 统计数据
        self.scan_count = 0
        self.last_scan_time = self.get_clock().now()
        
        # 串口连接状态
        self.serial_connected = False
        self.serial_thread = None
        
        # 启动串口读取线程
        self.start_serial_thread()
        
        # 创建定时器，定期发布扫描数据
        self.create_timer(0.05, self.publish_scan)  # 20Hz发布频率
        
        self.get_logger().info(f'N10激光雷达驱动节点已启动')
        self.get_logger().info(f'串口: {self.port_name}, 波特率: {self.baud_rate}')
        self.get_logger().info(f'坐标系: {self.frame_id}')
    
    def start_serial_thread(self):
        """启动串口数据读取线程"""
        self.serial_thread = threading.Thread(target=self.parse_lidar_data, daemon=True)
        self.serial_thread.start()
    
    def parse_lidar_data(self):
        """串口数据读取和解析线程（核心功能）"""
        try:
            # 打开串口
            ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            ser.reset_input_buffer()
            
            self.serial_connected = True
            self.get_logger().info('✅ 雷达硬件连接成功，开始解析数据...')
            
            # 清空数据缓冲区
            self.lidar_data_360 = [0.0] * 360
            
            # 主循环：持续读取和解析数据
            while rclpy.ok() and self.serial_connected:
                # 寻找帧头 0xA5 0x5A
                if ser.read(1) == b'\xA5' and ser.read(1) == b'\x5A':
                    # 读取56字节有效载荷
                    payload = ser.read(56)
                    
                    if len(payload) == 56 and payload[0] == 0x3A and payload[1] == 0x10:
                        # 解析起止角度
                        start_angle = (payload[3] << 8 | payload[2]) / 100.0
                        end_angle = (payload[53] << 8 | payload[52]) / 100.0
                        
                        # 处理跨越360度(0度)的情况
                        angle_diff = end_angle - start_angle
                        if angle_diff < 0:
                            angle_diff += 360.0
                        
                        angle_step = angle_diff / 15.0 if angle_diff > 0 else 0
                        
                        # 提取16个测距点
                        for i in range(16):
                            offset = 4 + i * 3
                            dist_l = payload[offset]
                            dist_h = payload[offset + 1]
                            distance_mm = (dist_h << 8) | dist_l
                            
                            # 计算当前点的具体角度，并四舍五入到整数
                            current_angle = (start_angle + i * angle_step) % 360
                            int_angle = int(current_angle + 0.5) % 360
                            
                            # 更新全局数组（过滤掉0值和过大的噪点）
                            if 0 < distance_mm < 15000:  # 限制在15米以内
                                # 毫米转换为米
                                self.lidar_data_360[int_angle] = distance_mm / 1000.0
                        
                        # 更新扫描计数
                        self.scan_count += 1
                        self.last_scan_time = self.get_clock().now()
            
        except serial.SerialException as e:
            self.get_logger().error(f'❌ 串口连接失败: {e}')
            self.serial_connected = False
        except Exception as e:
            self.get_logger().error(f'❌ 数据解析异常: {e}')
            self.serial_connected = False
    
    def publish_scan(self):
        """定期发布LaserScan消息"""
        if not self.serial_connected or self.scan_count == 0:
            return
        
        # 创建LaserScan消息
        scan_msg = LaserScan()
        
        # 设置消息头
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        
        # 设置激光雷达参数（N10雷达规格）
        scan_msg.angle_min = -math.pi  # -180度
        scan_msg.angle_max = math.pi   # 180度
        scan_msg.angle_increment = 2.0 * math.pi / 360.0  # 1度分辨率
        
        # 扫描时间（假设10Hz扫描频率）
        scan_msg.scan_time = 0.1
        scan_msg.time_increment = scan_msg.scan_time / 360.0
        
        # 设置距离范围
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        
        # 填充距离数据
        scan_msg.ranges = [0.0] * 360
        
        # 应用角度偏移和反转
        for i in range(360):
            if self.invert_frame:
                # 反转坐标系（180度旋转）
                src_idx = (i + 180) % 360
            else:
                src_idx = i
            
            # 应用角度偏移
            offset_idx = int((src_idx + self.angle_offset * 180.0 / math.pi) + 0.5) % 360
            
            distance = self.lidar_data_360[offset_idx]
            
            if distance > 0:
                # 确保距离在有效范围内
                if self.range_min <= distance <= self.range_max:
                    scan_msg.ranges[i] = distance
                else:
                    scan_msg.ranges[i] = float('inf')
            else:
                scan_msg.ranges[i] = float('inf')
        
        # 发布消息
        self.scan_pub.publish(scan_msg)
        
        # 每100次扫描打印一次统计信息
        if self.scan_count % 100 == 0:
            self.get_logger().debug(f'已处理 {self.scan_count} 次扫描，数据率: {1.0 / (self.get_clock().now() - self.last_scan_time).nanoseconds * 1e9:.1f} Hz')
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        self.serial_connected = False
        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # 创建驱动节点
    driver_node = N10LidarDriver()
    
    try:
        # 运行节点
        rclpy.spin(driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        driver_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()