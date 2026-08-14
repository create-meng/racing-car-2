#!/usr/bin/env python3
"""Keep dynamic laser obstacles, suppress returns explained by the static map."""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformException


class StaticWallScanFilter(Node):
    def __init__(self):
        super().__init__('static_wall_scan_filter')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('filtered_topic', '/scan_nav')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('match_tolerance', 0.10)
        self.declare_parameter('map_cell_radius', 1)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('max_tf_age', 0.20)
        self.declare_parameter('odom_frame', 'odom_combined')
        self.declare_parameter('max_map_odom_age', 2.0)

        self.map_msg = None
        self.last_report = self.get_clock().now()
        self.filtered = 0
        self.kept = 0
        self.near_raw = 0
        self.near_filtered = 0
        self.near_kept = 0
        self.near_detail = 'none'
        self.near_min = float('inf')
        self.processed = 0
        self.tf_split = 0
        self.tf_dropped = 0
        self.tf_map_odom_old = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        scan_topic = self.get_parameter('scan_topic').value
        filtered_topic = self.get_parameter('filtered_topic').value
        map_topic = self.get_parameter('map_topic').value
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(LaserScan, filtered_topic, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, map_topic, self.map_callback, map_qos)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'静态墙激光过滤启动: {scan_topic} -> {filtered_topic}, map={map_topic}')

    def map_callback(self, msg):
        self.map_msg = msg

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _transform(x, y, transform):
        t = transform.transform.translation
        q = transform.transform.rotation
        # 2D point rotation from quaternion, retaining the full TF translation.
        yaw = StaticWallScanFilter._yaw(q)
        return (t.x + math.cos(yaw) * x - math.sin(yaw) * y,
                t.y + math.sin(yaw) * x + math.cos(yaw) * y)

    @classmethod
    def _compose_2d(cls, first, second, parent, child, stamp):
        """Compose parent->middle and middle->child planar transforms."""
        a = first.transform.translation
        b = second.transform.translation
        ayaw = cls._yaw(first.transform.rotation)
        byaw = cls._yaw(second.transform.rotation)
        out = TransformStamped()
        out.header.frame_id = parent
        out.header.stamp = stamp.to_msg()
        out.child_frame_id = child
        out.transform.translation.x = a.x + math.cos(ayaw) * b.x - math.sin(ayaw) * b.y
        out.transform.translation.y = a.y + math.sin(ayaw) * b.x + math.cos(ayaw) * b.y
        out.transform.translation.z = a.z + b.z
        out.transform.rotation.z = math.sin((ayaw + byaw) * 0.5)
        out.transform.rotation.w = math.cos((ayaw + byaw) * 0.5)
        return out

    def _grid_xy(self, x, y):
        info = self.map_msg.info
        ox, oy = info.origin.position.x, info.origin.position.y
        oyaw = self._yaw(info.origin.orientation)
        dx, dy = x - ox, y - oy
        c, s = math.cos(oyaw), math.sin(oyaw)
        gx = (c * dx + s * dy) / info.resolution
        gy = (-s * dx + c * dy) / info.resolution
        return int(math.floor(gx)), int(math.floor(gy))

    def _occupied_near(self, x, y):
        if self.map_msg is None:
            return False
        info = self.map_msg.info
        gx, gy = self._grid_xy(x, y)
        radius = int(self.get_parameter('map_cell_radius').value)
        threshold = int(self.get_parameter('occupied_threshold').value)
        width, height = info.width, info.height
        data = self.map_msg.data
        for yy in range(gy - radius, gy + radius + 1):
            if yy < 0 or yy >= height:
                continue
            for xx in range(gx - radius, gx + radius + 1):
                if 0 <= xx < width and data[yy * width + xx] >= threshold:
                    return True
        return False

    def _first_static_hit(self, origin, angle, max_range):
        if self.map_msg is None:
            return None
        resolution = self.map_msg.info.resolution
        step = max(0.5 * resolution, 0.01)
        distance = step
        while distance <= max_range:
            if self._occupied_near(origin[0] + distance * math.cos(angle),
                                    origin[1] + distance * math.sin(angle)):
                return distance
            distance += step
        return None

    @staticmethod
    def _inside_footprint(x, y):
        # Same footprint as nav.yaml, with no extra safety margin.
        return -0.14 <= x <= 0.14 and -0.11 <= y <= 0.11

    def _lookup_scan_tf(self, scan):
        target = self.get_parameter('map_frame').value
        odom = self.get_parameter('odom_frame').value
        stamp = Time.from_msg(scan.header.stamp)
        # Query the continuously published odometry edge at the scan time,
        # then compose it with the newest AMCL map->odom correction.  A direct
        # map<-laser lookup extrapolates whenever AMCL has not published a new
        # map->odom sample for the current laser timestamp.
        odom_to_laser = self.tf_buffer.lookup_transform(
            odom, scan.header.frame_id, stamp, timeout=Duration(seconds=0.03))
        map_to_odom = self.tf_buffer.lookup_transform(
            target, odom, Time(), timeout=Duration(seconds=0.03))
        map_stamp = Time.from_msg(map_to_odom.header.stamp)
        map_age = (stamp.nanoseconds - map_stamp.nanoseconds) / 1e9
        if map_age < -float(self.get_parameter('max_tf_age').value):
            raise TransformException(
                f'map->odom TF is in the future by {-map_age:.3f}s')
        if map_age > float(self.get_parameter('max_map_odom_age').value):
            raise TransformException(
                f'map->odom TF is too old ({map_age:.3f}s)')
        return self._compose_2d(map_to_odom, odom_to_laser, target,
                                scan.header.frame_id, stamp), map_age

    def scan_callback(self, scan):
        # Until both map and TF are available, pass scans through unchanged.
        if self.map_msg is None:
            self.tf_dropped += 1
            self._maybe_report()
            return
        try:
            tf, map_age = self._lookup_scan_tf(scan)
            self.tf_split += 1
            if map_age > float(self.get_parameter('max_tf_age').value):
                self.tf_map_odom_old += 1
        except TransformException as exc:
            self.tf_dropped += 1
            self.get_logger().warn(f'等待完整激光 TF，丢弃本帧 /scan_nav: {exc}', throttle_duration_sec=5.0)
            self._maybe_report()
            return
        try:
            base_tf = self.tf_buffer.lookup_transform(
                'base_footprint', scan.header.frame_id,
                Time(), timeout=Duration(seconds=0.05))
        except TransformException:
            base_tf = None

        filtered = list(scan.ranges)
        laser_origin = self._transform(0.0, 0.0, tf)
        angle = scan.angle_min
        removed = 0
        kept = 0
        tolerance = float(self.get_parameter('match_tolerance').value)
        for i, value in enumerate(scan.ranges):
            if not math.isfinite(value) or value < scan.range_min or value > scan.range_max:
                angle += scan.angle_increment
                continue
            endpoint_laser = (value * math.cos(angle), value * math.sin(angle))
            endpoint_map = self._transform(endpoint_laser[0], endpoint_laser[1], tf)
            endpoint_base = (None if base_tf is None else
                             self._transform(endpoint_laser[0], endpoint_laser[1], base_tf))
            static_hit = self._first_static_hit(laser_origin, angle + self._yaw(tf.transform.rotation), value + tolerance)
            # A scan return is static-wall-consistent only when the first map wall
            # lies at the same range. A cone in front of a wall remains dynamic.
            static_match = (static_hit is not None and
                            abs(static_hit - value) <= tolerance and
                            self._occupied_near(*endpoint_map))
            near_robot = endpoint_base is not None and self._inside_footprint(*endpoint_base)
            if near_robot:
                self.near_raw += 1
            if static_match:
                filtered[i] = float('inf')
                removed += 1
                if near_robot:
                    self.near_filtered += 1
            else:
                kept += 1
                if near_robot:
                    self.near_kept += 1
            if near_robot and value < self.near_min:
                action = 'FILTER_STATIC_WALL' if static_match else 'KEEP_DYNAMIC'
                base_text = f'base=({endpoint_base[0]:.3f},{endpoint_base[1]:.3f})'
                map_text = f'map=({endpoint_map[0]:.3f},{endpoint_map[1]:.3f})'
                hit_text = 'none' if static_hit is None else f'{static_hit:.3f}m'
                self.near_detail = f'{value:.3f}m {action} {base_text} {map_text} static_hit={hit_text}'
                self.near_min = value
            angle += scan.angle_increment

        out = LaserScan()
        out = scan
        out.ranges = filtered
        self.pub.publish(out)
        self.processed += 1
        self.filtered += removed
        self.kept += kept
        self._maybe_report()

    def _maybe_report(self):
        now = self.get_clock().now()
        if (now - self.last_report).nanoseconds >= 2_000_000_000:
            self.get_logger().info(
                f'过滤统计: processed={self.processed} '
                f'tf_split={self.tf_split} tf_dropped={self.tf_dropped} '
                f'map_odom_old={self.tf_map_odom_old} '
                f'static_wall={self.filtered} dynamic_kept={self.kept} | '
                f'车体footprint内: raw={self.near_raw} '
                f'filtered={self.near_filtered} kept={self.near_kept} | '
                f'最近近点: {self.near_detail}')
            self.processed = self.tf_split = self.tf_dropped = self.tf_map_odom_old = 0
            self.filtered = self.kept = 0
            self.near_raw = self.near_filtered = self.near_kept = 0
            self.near_detail = 'none'
            self.near_min = float('inf')
            self.last_report = now


def main(args=None):
    rclpy.init(args=args)
    node = StaticWallScanFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
