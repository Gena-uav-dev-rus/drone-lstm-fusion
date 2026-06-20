#!/usr/bin/env python3
"""
Простой скрипт проверки дрейфа — сравнивает /global_odometry (EKF estimate)
с /ground_truth/x500_mono_cam_0/odometry (истина) в реальном времени.

EKF и Gazebo world frame имеют разные нули отсчёта (EKF стартует с (0,0,0)
своей внутренней системы, Gazebo использует абсолютные world координаты).
Этот скрипт компенсирует начальный offset автоматически по первой паре
синхронных измерений — это чисто для удобства ВИЗУАЛИЗАЦИИ дрейфа,
сам EKF/global_fusion_node не модифицируется и offset не "видит".

Запуск:
  source ~/drone-lstm-fusion/ros2_ws/install/setup.bash
  python3 ~/drone-lstm-fusion/scripts/check_drift.py
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time


class DriftChecker(Node):
    def __init__(self):
        super().__init__('drift_checker')

        self.latest_gt = None
        self.latest_fused = None

        self.offset = None  # (dx, dy, dz) — gt - fused в момент калибровки

        self.sub_gt = self.create_subscription(
            Odometry, '/ground_truth/x500_mono_cam_0/odometry', self.gt_cb, 10)
        self.sub_fused = self.create_subscription(
            Odometry, '/global_odometry', self.fused_cb, 10)

        self.timer = self.create_timer(1.0, self.print_drift)

        self.get_logger().info('DriftChecker started. Calibrating offset on first sample...')
        print(f"{'time':>8} {'gt_x':>8} {'gt_y':>8} {'gt_z':>8} | "
              f"{'fused_x':>8} {'fused_y':>8} {'fused_z':>8} | {'error_m':>8}")

    def gt_cb(self, msg):
        self.latest_gt = msg.pose.pose.position

    def fused_cb(self, msg):
        self.latest_fused = msg.pose.pose.position

    def print_drift(self):
        if self.latest_gt is None or self.latest_fused is None:
            print("Waiting for both topics...")
            return

        gt = self.latest_gt
        fused = self.latest_fused

        # Калибруем offset один раз, по первой доступной паре измерений
        if self.offset is None:
            self.offset = (gt.x - fused.x, gt.y - fused.y, gt.z - fused.z)
            self.get_logger().info(
                f'Offset calibrated: dx={self.offset[0]:.2f} '
                f'dy={self.offset[1]:.2f} dz={self.offset[2]:.2f}')

        # Применяем offset к fused перед сравнением
        fused_x = fused.x + self.offset[0]
        fused_y = fused.y + self.offset[1]
        fused_z = fused.z + self.offset[2]

        error = math.sqrt(
            (gt.x - fused_x)**2 +
            (gt.y - fused_y)**2 +
            (gt.z - fused_z)**2
        )

        t = time.strftime('%H:%M:%S')
        print(f"{t:>8} {gt.x:8.2f} {gt.y:8.2f} {gt.z:8.2f} | "
              f"{fused_x:8.2f} {fused_y:8.2f} {fused_z:8.2f} | {error:8.3f}")


def main():
    rclpy.init()
    node = DriftChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
