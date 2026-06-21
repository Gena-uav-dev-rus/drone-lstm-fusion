#!/usr/bin/env python3
"""
Записывает пары (ground_truth_z, vio_z) в реальном времени для вычисления
scale factor VIO по отношению к истинной высоте. Печатает CSV-строки,
можно перенаправить в файл и затем посчитать линейную регрессию
(vio_z = scale * gt_z + offset) для получения точного scale factor.

Запуск:
  source ~/drone-lstm-fusion/ros2_ws/install/setup.bash
  python3 ~/drone-lstm-fusion/scripts/measure_vio_scale.py | tee /tmp/vio_scale_data.csv
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time


class ScaleMeasurer(Node):
    def __init__(self):
        super().__init__('vio_scale_measurer')

        self.latest_gt_z = None
        self.latest_vio_z = None

        self.sub_gt = self.create_subscription(
            Odometry, '/ground_truth/x500_mono_cam_0/odometry', self.gt_cb, 10)
        self.sub_vio = self.create_subscription(
            Odometry, '/vio/odometry', self.vio_cb, 10)

        self.timer = self.create_timer(0.5, self.print_pair)
        print("timestamp,gt_z,vio_z")

    def gt_cb(self, msg):
        self.latest_gt_z = msg.pose.pose.position.z

    def vio_cb(self, msg):
        self.latest_vio_z = msg.pose.pose.position.z

    def print_pair(self):
        if self.latest_gt_z is None or self.latest_vio_z is None:
            return
        t = time.time()
        print(f"{t:.2f},{self.latest_gt_z:.4f},{self.latest_vio_z:.4f}")


def main():
    rclpy.init()
    node = ScaleMeasurer()
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
