#!/usr/bin/env python3
"""
Bridge: /camera (Gazebo) → /mono_py_driver/img_msg (ros2_orb_slam3)
+ отправляет handshake с именем конфига
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String

class GazeboMonoDriver(Node):
    def __init__(self):
        super().__init__('gazebo_mono_driver')
        
        self.handshake_sent = False
        
        # Подписываемся на Gazebo камеру
        self.sub_img = self.create_subscription(
            Image, '/camera', self.img_callback, 10)
        
        # Подписываемся на ACK от ORB-SLAM3
        self.sub_ack = self.create_subscription(
            String, '/mono_py_driver/exp_settings_ack', self.ack_callback, 10)

        # Публикуем в формат ros2_orb_slam3
        self.pub_img = self.create_publisher(
            Image, '/mono_py_driver/img_msg', 10)
        self.pub_ts = self.create_publisher(
            Float64, '/mono_py_driver/timestep_msg', 10)
        self.pub_config = self.create_publisher(
            String, '/mono_py_driver/experiment_settings', 10)

        # Таймер для handshake — шлём каждую секунду пока не получим ACK
        self.timer = self.create_timer(1.0, self.send_handshake)
        
        self.get_logger().info('GazeboMonoDriver started, waiting for ORB-SLAM3...')

    def send_handshake(self):
        if not self.handshake_sent:
            msg = String()
            msg.data = "gazebo_mono"  # имя конфига — ORB-SLAM3 добавит .yaml
            self.pub_config.publish(msg)
            self.get_logger().info('Sent handshake: gazebo_mono')

    def ack_callback(self, msg):
        self.handshake_sent = True
        self.timer.cancel()
        self.get_logger().info('Handshake complete! ORB-SLAM3 ready.')

    def img_callback(self, msg):
        self.pub_img.publish(msg)
        ts = Float64()
        ts.data = self.get_clock().now().nanoseconds / 1e9
        self.pub_ts.publish(ts)

def main():
    rclpy.init()
    rclpy.spin(GazeboMonoDriver())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
