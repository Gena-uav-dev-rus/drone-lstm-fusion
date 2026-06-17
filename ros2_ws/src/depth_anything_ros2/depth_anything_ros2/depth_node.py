#!/usr/bin/env python3
"""
Depth Anything V2 metric — ROS2 node.
Подписывается на /camera (RGB), публикует metric depth.

ВАЖНО: dev-решение для desktop, запускается в venv с --system-site-packages.
Для Jetson/production — заменить на TensorRT C++ ноду (см. docs/install_notes.md).
"""
import sys
import os

# Путь к репозиторию Depth Anything V2 metric_depth
DEPTH_ANYTHING_PATH = os.path.expanduser("~/Depth-Anything-V2/metric_depth")
sys.path.insert(0, DEPTH_ANYTHING_PATH)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
import numpy as np
import torch

from depth_anything_v2.dpt import DepthAnythingV2

MODEL_CONFIGS = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
}

CHECKPOINT_PATH = os.path.join(
    DEPTH_ANYTHING_PATH, "checkpoints", "depth_anything_v2_metric_vkitti_vits.pth"
)


class DepthAnythingNode(Node):
    def __init__(self):
        super().__init__('depth_anything_node')

        self.get_logger().info('Loading Depth Anything V2 model...')

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Using device: {self.device}')

        self.model = DepthAnythingV2(**{**MODEL_CONFIGS['vits'], 'max_depth': 80})
        self.model.load_state_dict(torch.load(CHECKPOINT_PATH, map_location='cpu'))
        self.model = self.model.to(self.device).eval()

        self.get_logger().info('Model loaded successfully.')

        # Подписка на камеру дрона
        self.sub_img = self.create_subscription(Image, '/camera', self.img_callback, 1)

        # Публикуем расстояние до точки прямо под дроном (центр нижней части кадра,
        # так как камера смотрит вперёд-вниз под 45°)
        self.pub_altitude = self.create_publisher(Float32, '/depth/altitude', 10)

        # Публикуем визуализацию depth map (для отладки/RViz)
        self.pub_depth_image = self.create_publisher(Image, '/depth/image', 10)

        self.frame_count = 0
        self.get_logger().info('DepthAnythingNode ready, waiting for frames...')

    def img_callback(self, msg: Image):
        try:
            # Ручная конвертация без cv_bridge (избегаем версии конфликтов)
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        with torch.no_grad():
            depth = self.model.infer_image(img_bgr)  # HxW float32, метры

        # Altitude — берём медиану нижней трети кадра (то что под дроном с учётом угла камеры)
        h, w = depth.shape
        bottom_third = depth[int(h * 0.66):, :]
        altitude_estimate = float(np.median(bottom_third))

        alt_msg = Float32()
        alt_msg.data = altitude_estimate
        self.pub_altitude.publish(alt_msg)

        # Публикуем визуализацию каждый 5-й кадр (не грузим топик зря)
        self.frame_count += 1
        if self.frame_count % 5 == 0:
            depth_norm = (depth / max(depth.max(), 1e-6) * 255).astype(np.uint8)
            depth_vis = cv2.applyColorMap(depth_norm, cv2.COLORMAP_INFERNO)

            depth_msg = Image()
            depth_msg.header = msg.header
            depth_msg.height = depth_vis.shape[0]
            depth_msg.width = depth_vis.shape[1]
            depth_msg.encoding = 'bgr8'
            depth_msg.is_bigendian = 0
            depth_msg.step = depth_vis.shape[1] * 3
            depth_msg.data = depth_vis.tobytes()
            self.pub_depth_image.publish(depth_msg)


def main():
    rclpy.init()
    node = DepthAnythingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
