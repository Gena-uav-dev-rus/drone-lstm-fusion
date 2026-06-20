#!/usr/bin/env python3
"""
VIO LSTM noise estimator — inference нода.

Подписывается на /vio/odometry, поддерживает скользящее окно истории,
прогоняет через предобученную LSTM, публикует variance в /lstm_noise/vio_variance.

Фичи (порядок ДОЛЖЕН совпадать с train_lstm.py): vio_x, vio_y, vio_z,
vio_qx, vio_qy, vio_qz, vio_qw
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


class VioNoiseNode(Node):
    def __init__(self):
        super().__init__('vio_noise_node')

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Loading VIO LSTM model on {self.device}...')

        self.model, self.seq_len = load_model(
            os.path.join(MODELS_DIR, 'lstm_vio.pt'), self.device)

        with open(os.path.join(MODELS_DIR, 'scaler_vio.pkl'), 'rb') as f:
            self.scaler = pickle.load(f)

        self.get_logger().info(f'Model loaded, seq_len={self.seq_len}')

        self.history = deque(maxlen=self.seq_len)

        self.sub_vio = self.create_subscription(
            Odometry, '/vio/odometry', self.vio_callback, 10)

        self.pub_variance = self.create_publisher(Float32, '/lstm_noise/vio_variance', 10)

        self.get_logger().info('VioNoiseNode ready, waiting for VIO data...')

    def vio_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        features = np.array([
            p.x, p.y, p.z, q.x, q.y, q.z, q.w
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
    node = VioNoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
