#!/usr/bin/env python3
"""
GPS LSTM noise estimator — inference нода.

Подписывается на сырые GPS измерения, поддерживает скользящее окно истории
(SEQ_LEN последних измерений), прогоняет через предобученную LSTM,
публикует предсказанную variance в /lstm_noise/gps_variance.

ВАЖНО: фичи на входе ДОЛЖНЫ точно соответствовать порядку колонок
из training/train_lstm.py: gps_north, gps_east, gps_down, gps_vn, gps_ve, gps_vd,
gps_fix_type, gps_satellites — нормализованные тем же scaler.
"""
import os
import pickle
import numpy as np
import torch
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from px4_msgs.msg import SensorGps

from lstm_noise_estimator.model import load_model

MODELS_DIR = os.path.expanduser("~/drone-lstm-fusion/training/models")


class GpsNoiseNode(Node):
    def __init__(self):
        super().__init__('gps_noise_node')

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Loading GPS LSTM model on {self.device}...')

        self.model, self.seq_len = load_model(
            os.path.join(MODELS_DIR, 'lstm_gps.pt'), self.device)

        with open(os.path.join(MODELS_DIR, 'scaler_gps.pkl'), 'rb') as f:
            self.scaler = pickle.load(f)

        self.get_logger().info(f'Model loaded, seq_len={self.seq_len}')

        self.history = deque(maxlen=self.seq_len)

        # GPS origin для конвертации lat/lon -> локальные метры (как в датасете)
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None

        px4_qos = QoSProfile(depth=10)
        px4_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.sub_gps = self.create_subscription(
            SensorGps, '/fmu/out/vehicle_gps_position', self.gps_callback, px4_qos)

        self.pub_variance = self.create_publisher(Float32, '/lstm_noise/gps_variance', 10)

        self.get_logger().info('GpsNoiseNode ready, waiting for GPS data...')

    def gps_callback(self, msg: SensorGps):
        if self.origin_lat is None:
            self.origin_lat = msg.latitude_deg
            self.origin_lon = msg.longitude_deg
            self.origin_alt = msg.altitude_msl_m
            self.get_logger().info(
                f'GPS origin set: lat={self.origin_lat:.6f} lon={self.origin_lon:.6f}')
            return

        EARTH_R = 6378137.0
        lat_rad = np.deg2rad(self.origin_lat)

        north = np.deg2rad(msg.latitude_deg - self.origin_lat) * EARTH_R
        east = np.deg2rad(msg.longitude_deg - self.origin_lon) * EARTH_R * np.cos(lat_rad)
        down = -(msg.altitude_msl_m - self.origin_alt)

        features = np.array([
            north, east, down,
            msg.vel_n_m_s, msg.vel_e_m_s, msg.vel_d_m_s,
            float(msg.fix_type), float(msg.satellites_used)
        ], dtype=np.float32)

        self.history.append(features)

        if len(self.history) < self.seq_len:
            return  # ждём заполнения окна истории

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
    node = GpsNoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
