"""obstacle_marker_publisher.py — 障碍物可视化模块（圆圈标记）

用于 Stage1 和 Stage2 发布障碍物的 rviz2 可视化 Marker。
每个障碍物显示为半径 13cm 的圆柱体。

用法：
    from racing_common.obstacle_marker_publisher import ObstacleMarkerPublisher
    
    # 初始化
    marker_pub = ObstacleMarkerPublisher(node, topic='/obstacle_markers', frame_id='base_link')
    
    # 从聚类发布（Stage1 用）
    clusters = [[(x1,y1,d1), (x2,y2,d2), ...], ...]
    marker_pub.publish_from_clusters(clusters, color='red')
    
    # 从点列表发布（Stage2 用）
    points = [(x1, y1), (x2, y2), ...]
    marker_pub.publish_from_points(points, color='red')
    
    # 清空所有 markers
    marker_pub.clear()
"""

from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


class ObstacleMarkerPublisher:
    """障碍物 Marker 发布器
    
    发布圆柱体 Marker 表示障碍物，用于 rviz2 可视化调试。
    """
    
    def __init__(self, node, topic='/obstacle_markers', frame_id='base_link', radius=0.13):
        """初始化
        
        Args:
            node: ROS 2 节点实例
            topic: Marker 话题名称
            frame_id: 坐标系 ID（通常是 'base_link' 或 'laser'）
            radius: 障碍物圆圈半径（m），默认 0.13m = 13cm
        """
        self.node = node
        self.frame_id = frame_id
        self.radius = radius
        self.diameter = radius * 2.0
        # 使用 rviz2 兼容的 QoS
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # 让 late joiner 也能收到
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.marker_pub = node.create_publisher(MarkerArray, topic, qos)
        self._logger = node.get_logger()
        self._is_cleared = True  # 防抖标志：True=已清空，False=有 markers
        self._last_marker_count = 0  # 上次发布的 marker 数量
        self._logger.debug(
            f'ObstacleMarkerPublisher initialized: topic={topic}, '
            f'frame={frame_id}, radius={radius:.3f}m'
        )
    
    def publish_from_clusters(self, clusters: list, color='red', max_markers=20):
        """从聚类列表发布 Markers，无闪烁更新
        
        用固定 ID 池更新 marker 位置，数量变化时自动 DELETE 多余 marker。
        不依赖 DELETEALL，避免闪烁。
        
        Args:
            clusters: 聚类列表 [[(x,y,dist), ...], ...]
            color: 颜色名称 'red' / 'yellow' / 'green'
            max_markers: 最大 marker 数量（防无限增长）
        """
        markers = MarkerArray()
        stamp = self.node.get_clock().now().to_msg()
        
        valid_count = 0
        for i, cluster in enumerate(clusters):
            if not cluster or i >= max_markers:
                continue
            try:
                cx = sum(p[0] for p in cluster) / len(cluster)
                cy = sum(p[1] for p in cluster) / len(cluster)
                markers.markers.append(
                    self._make_cylinder(i, cx, cy, color)
                )
                valid_count += 1
            except (IndexError, ZeroDivisionError, TypeError) as e:
                self._logger.warn(f'Skipping cluster {i}: {e}')
                continue
        
        # 清理比当前多的旧 marker（当聚类数减少时）
        prev_count = self._last_marker_count
        if valid_count < prev_count:
            for i in range(valid_count, prev_count):
                m = Marker()
                m.header.frame_id = self.frame_id
                m.header.stamp = stamp
                m.ns = 'obstacles'
                m.id = i
                m.action = Marker.DELETE
                markers.markers.append(m)
        
        self.marker_pub.publish(markers)
        self._is_cleared = False
        self._last_marker_count = valid_count
        if valid_count > 0:
            self._logger.debug(f'Published {valid_count} obstacle markers from clusters')
    
    def publish_from_points(self, points: list, color='red'):
        """从点列表发布 Markers（Stage2 用）
        
        Args:
            points: 点列表 [(x1, y1), (x2, y2), ...]
            color: 颜色名称 'red' / 'yellow' / 'green'
        """
        if not points:
            self.clear()
            return
        
        markers = MarkerArray()
        valid_count = 0
        
        for i, point in enumerate(points):
            try:
                x, y = point[0], point[1]
                markers.markers.append(
                    self._make_cylinder(valid_count, x, y, color)
                )
                valid_count += 1
            except (IndexError, TypeError) as e:
                self._logger.warn(f'Skipping invalid point {i}: {e}')
                continue
        
        if valid_count > 0:
            self.marker_pub.publish(markers)
            self._is_cleared = False
            self._last_marker_count = valid_count
            self._logger.debug(f'Published {valid_count} obstacle markers from points')
        else:
            self.clear()
    
    def clear(self):
        """清空所有 Markers（带防抖）"""
        if self._is_cleared:
            # 已经清空过，避免重复发布 DELETEALL
            return
        
        markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.frame_id = self.frame_id
        delete_marker.header.stamp = self.node.get_clock().now().to_msg()
        delete_marker.ns = 'obstacles'
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)
        self.marker_pub.publish(markers)
        
        self._is_cleared = True
        self._last_marker_count = 0
        self._logger.debug('Cleared all obstacle markers')
    
    def _make_cylinder(self, marker_id: int, x: float, y: float, color: str):
        """创建单个圆柱体 Marker
        
        Args:
            marker_id: Marker ID
            x: X 坐标（m）
            y: Y 坐标（m）
            color: 颜色名称
        
        Returns:
            Marker 消息
        """
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = 'obstacles'
        m.id = marker_id
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        
        # 位置：扁平圆盘，贴地显示
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.01  # 紧贴地面
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        
        # 尺寸：扁平圆盘
        m.scale.x = self.diameter
        m.scale.y = self.diameter
        m.scale.z = 0.02  # 扁平
        
        # 颜色
        if color == 'red':
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.0, 0.9
        elif color == 'yellow':
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 0.8
        elif color == 'green':
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.8
        elif color == 'orange':
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.6
        else:  # 默认白色
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 1.0, 0.5
        
        # 生命周期：0 = 不超时，由 clear() 手动删除
        m.lifetime = Duration(sec=5, nanosec=0)  # 5s 长寿命，抗闪烁
        
        return m
