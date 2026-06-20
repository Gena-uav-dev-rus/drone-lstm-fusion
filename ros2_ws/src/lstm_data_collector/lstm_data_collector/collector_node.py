#!/usr/bin/env python3
"""
Dataset collector для обучения LSTM noise estimator.

Синхронно записывает:
  - ground truth позицию/скорость (из Gazebo plugin)
  - сырые GPS измерения (из PX4)
  - VIO одометрию (из ORB-SLAM3)
  - depth altitude (из Depth Anything)

Выход: CSV файл с временными метками и всеми измерениями.
Запуск: ros2 run lstm_data_collector collector_node.py --ros-args -p output_file:=/tmp/dataset.csv
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from px4_msgs.msg import SensorGps

import csv
import os
import time
from collections import deque


class DataCollectorNode(Node):
    def __init__(self):
        super().__init__('lstm_data_collector')

        self.declare_parameter('output_file', '/tmp/lstm_dataset.csv')
        self.declare_parameter('max_samples', 50000)

        output_file = self.get_parameter('output_file').value
        self.max_samples = self.get_parameter('max_samples').value

        # Последние известные значения каждого источника (не синхронизируем жёстко,
        # записываем по ground truth callback с последними доступными значениями)
        self.latest_gps = None
        self.latest_vio = None
        self.latest_depth = None
        self.latest_gt = None

        # Ground truth — главный триггер записи строки в CSV
        self.sub_gt = self.create_subscription(
            Odometry,
            '/ground_truth/x500_mono_cam_0/odometry',
            self.gt_callback, 10)

        # GPS от PX4 (BEST_EFFORT QoS)
        px4_qos = QoSProfile(depth=10)
        px4_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.sub_gps = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.gps_callback, px4_qos)

        self.sub_vio = self.create_subscription(
            Odometry, '/vio/odometry', self.vio_callback, 10)

        self.sub_depth = self.create_subscription(
            Float32, '/depth/altitude', self.depth_callback, 10)

        # Открываем CSV файл
        os.makedirs(os.path.dirname(output_file) if os.path.dirname(output_file) else '.', exist_ok=True)
        self.csv_file = open(output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Заголовки CSV
        self.csv_writer.writerow([
            'timestamp',
            # Ground truth
            'gt_x', 'gt_y', 'gt_z',
            'gt_vx', 'gt_vy', 'gt_vz',
            'gt_qx', 'gt_qy', 'gt_qz', 'gt_qw',
            # GPS raw
            'gps_lat', 'gps_lon', 'gps_alt',
            'gps_vn', 'gps_ve', 'gps_vd',
            'gps_fix_type', 'gps_satellites',
            # VIO raw
            'vio_x', 'vio_y', 'vio_z',
            'vio_qx', 'vio_qy', 'vio_qz', 'vio_qw',
            # Depth
            'depth_altitude',
        ])

        self.sample_count = 0
        self.get_logger().info(f'DataCollector started, writing to {output_file}')
        self.get_logger().info(f'Max samples: {self.max_samples}')

    def gps_callback(self, msg):
        self.latest_gps = msg

    def vio_callback(self, msg):
        self.latest_vio = msg

    def depth_callback(self, msg):
        self.latest_depth = msg

    def gt_callback(self, msg):
        if self.sample_count >= self.max_samples:
            return

        # Пропускаем если нет хотя бы одного измерения от каждого источника
        if self.latest_gps is None or self.latest_vio is None or self.latest_depth is None:
            return

        ts = self.get_clock().now().nanoseconds / 1e9

        gt = msg.pose.pose
        gt_twist = msg.twist.twist
        gps = self.latest_gps
        vio = self.latest_vio.pose.pose
        depth = self.latest_depth.data

        row = [
            ts,
            # Ground truth позиция
            gt.position.x, gt.position.y, gt.position.z,
            gt_twist.linear.x, gt_twist.linear.y, gt_twist.linear.z,
            gt.orientation.x, gt.orientation.y, gt.orientation.z, gt.orientation.w,
            # GPS
            gps.latitude_deg, gps.longitude_deg, gps.altitude_msl_m,
            gps.vel_n_m_s, gps.vel_e_m_s, gps.vel_d_m_s,
            gps.fix_type, gps.satellites_used,
            # VIO
            vio.position.x, vio.position.y, vio.position.z,
            vio.orientation.x, vio.orientation.y, vio.orientation.z, vio.orientation.w,
            # Depth
            depth,
        ]

        self.csv_writer.writerow(row)
        self.sample_count += 1

        if self.sample_count % 100 == 0:
            self.csv_file.flush()
            self.get_logger().info(f'Collected {self.sample_count}/{self.max_samples} samples')

        if self.sample_count >= self.max_samples:
            self.get_logger().info('Dataset collection complete!')
            self.csv_file.close()

    def destroy_node(self):
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = DataCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
