#!/usr/bin/env python3
"""
Прямой мост VIO -> PX4 vehicle_visual_odometry.
Никакого EKF, никакого fusion — чистые данные ORB-SLAM3 напрямую в PX4.
Цель: проверить что сам канал vehicle_visual_odometry работает без проблем
      которые могли быть вызваны нашим GlobalFusionNode.

Запуск:
  source ~/drone-lstm-fusion/ros2_ws/install/setup.bash
  python3 ~/drone-lstm-fusion/scripts/vio_to_px4.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry, TimesyncStatus


class VioToPx4Bridge(Node):
    def __init__(self):
        super().__init__('vio_to_px4_bridge')

        self.px4_time_offset_us = 0
        self.timesync_received = False
        self.vio_count = 0  # счётчик полученных VIO сообщений

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_timesync = self.create_subscription(
            TimesyncStatus, '/fmu/out/timesync_status',
            self.timesync_callback, px4_qos)

        self.sub_vio = self.create_subscription(
            Odometry, '/vio/odometry',
            self.vio_callback, 10)

        self.pub_px4 = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', px4_qos)

        self.get_logger().info('VioToPx4Bridge started, waiting for timesync and VIO...')

    def timesync_callback(self, msg: TimesyncStatus):
        self.px4_time_offset_us = msg.estimated_offset
        if not self.timesync_received:
            self.timesync_received = True
            self.get_logger().info(
                f'Timesync received, offset={self.px4_time_offset_us} us')

    def vio_callback(self, msg: Odometry):
        if not self.timesync_received:
            return

        self.vio_count += 1

        ros_time_us = self.get_clock().now().nanoseconds // 1000
        px4_timestamp = ros_time_us + self.px4_time_offset_us

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        px4_msg = VehicleOdometry()
        px4_msg.timestamp = px4_timestamp
        px4_msg.timestamp_sample = px4_timestamp
        px4_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        px4_msg.position = [float(p.x), float(p.y), float(p.z)]
        px4_msg.q = [float(q.w), float(q.x), float(q.y), float(q.z)]
        px4_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        px4_msg.velocity = [float('nan'), float('nan'), float('nan')]
        px4_msg.angular_velocity = [float('nan'), float('nan'), float('nan')]
        # Консервативные variance — PX4 будет меньше доверять нашему VIO
        # и больше полагаться на GPS при fusion внутри EKF2
        px4_msg.position_variance = [0.1, 0.1, 0.1]
        px4_msg.orientation_variance = [0.05, 0.05, 0.05]
        px4_msg.velocity_variance = [float('nan'), float('nan'), float('nan')]
        px4_msg.quality = 80
        px4_msg.reset_counter = 0

        self.pub_px4.publish(px4_msg)

        if self.vio_count % 50 == 1:
            self.get_logger().info(
                f'VIO->PX4 #{self.vio_count}: pos=({p.x:.2f},{p.y:.2f},{p.z:.2f})')


def main():
    rclpy.init()
    node = VioToPx4Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
