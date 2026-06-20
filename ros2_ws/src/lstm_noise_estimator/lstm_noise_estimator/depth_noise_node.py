#!/usr/bin/env python3
"""
Depth LSTM noise estimator — inference нода.

Подписывается на /depth/altitude, поддерживает скользящее окно истории,
прогоняет через предобученную LSTM, публикует variance в /lstm_noise/depth_variance.

Фичи (порядок ДОЛЖЕН совпадать с train_lstm.py): depth_altitude, gt_vx, gt_vy, gt_vz
ПРИМЕЧАНИЕ: в датасете для обучения использовалась скорость из ground truth как
context-фича (показывающая что depth обычно хуже при быстром движении). В runtime
ground truth недоступен — заменяем на скорость из /global_odometry (EKF estimate).
"""
import os
import pickle
import numpy as np
import torch
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

from lstm_noise_estimator.model import load_model

MODELS_DIR = os.path.expanduser("~/drone-lstm-fusion/training/models")


class DepthNoiseNode(Node):
    def __init__(self):
        super().__init__('depth_noise_node')

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Loading Depth LSTM model on {self.device}...')

        self.model, self.seq_len = load_model(
            os.path.join(MODELS_DIR, 'lstm_depth.pt'), self.device)

        with open(os.path.join(MODELS_DIR, 'scaler_depth.pkl'), 'rb') as f:
            self.scaler = pickle.load(f)

        self.get_logger().info(f'Model loaded, seq_len={self.seq_len}')

        self.history = deque(maxlen=self.seq_len)

        # Последняя известная скорость (из EKF estimate, не ground truth — недоступен в runtime)
        self.latest_velocity = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        self.sub_depth = self.create_subscription(
            Float32, '/depth/altitude', self.depth_callback, 10)

        self.sub_odom = self.create_subscription(
            Odometry, '/global_odometry', self.odom_callback, 10)

        self.pub_variance = self.create_publisher(Float32, '/lstm_noise/depth_variance', 10)

        self.get_logger().info('DepthNoiseNode ready, waiting for depth data...')

    def odom_callback(self, msg: Odometry):
        v = msg.twist.twist.linear
        self.latest_velocity = np.array([v.x, v.y, v.z], dtype=np.float32)

    def depth_callback(self, msg: Float32):
        features = np.array([
            msg.data,
            self.latest_velocity[0], self.latest_velocity[1], self.latest_velocity[2]
        ], dtype=np.float32)

        self.history.append(features)

        if len(self.history) < self.seq_len:
            return

        seq = np.array(self.history)
        seq_scaled = self.scaler.transform(seq)
        x = torch.tensor(seq_scaled, dtype=torch.float32).unsqueeze(0).to(self.device)

        with torch.no_grad():
            variance = self.model(x).item()

        msg_out = Float32()
        msg_out.data = float(variance)
        self.pub_variance.publish(msg_out)


def main():
    rclpy.init()
    node = DepthNoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
