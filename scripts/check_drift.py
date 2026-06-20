#!/usr/bin/env python3
"""
Простой скрипт проверки дрейфа — сравнивает /global_odometry (EKF estimate)
с /ground_truth/x500_mono_cam_0/odometry (истина) в реальном времени.

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

        self.sub_gt = self.create_subscription(
            Odometry, '/ground_truth/x500_mono_cam_0/odometry', self.gt_cb, 10)
        self.sub_fused = self.create_subscription(
            Odometry, '/global_odometry', self.fused_cb, 10)

        self.timer = self.create_timer(1.0, self.print_drift)

        self.get_logger().info('DriftChecker started. Printing drift every 1 sec...')
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

        # ВАЖНО: global_odometry — EKF state, который интегрирует IMU в своей
        # собственной world frame, начинающейся с (0,0,0) в момент старта ноды.
        # Ground truth — Gazebo world frame абсолютная позиция. Они совпадают
        # по ориентации осей (NED-подобная локальная фрейм EKF), но имеют разный
        # ноль отсчёта если EKF стартовал не из точки (0,0,0) Gazebo.
        error = math.sqrt(
            (gt.x - fused.x)**2 +
            (gt.y - fused.y)**2 +
            (gt.z - fused.z)**2
        )

        t = time.strftime('%H:%M:%S')
        print(f"{t:>8} {gt.x:8.2f} {gt.y:8.2f} {gt.z:8.2f} | "
              f"{fused.x:8.2f} {fused.y:8.2f} {fused.z:8.2f} | {error:8.3f}")


def main():
    rclpy.init()
    node = DriftChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
