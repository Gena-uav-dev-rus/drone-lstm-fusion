#!/usr/bin/env python3
"""
Bridge: /camera (Gazebo) → /mono_py_driver/img_msg (ros2_orb_slam3)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

class GazeboMonoDriver(Node):
    def __init__(self):
        super().__init__('gazebo_mono_driver')
        
        # Подписываемся на Gazebo камеру
        self.sub_img = self.create_subscription(
            Image, '/camera', self.img_callback, 10)
        
        # Публикуем в формате который ожидает ros2_orb_slam3
        self.pub_img = self.create_publisher(
            Image, '/mono_py_driver/img_msg', 10)
        self.pub_ts = self.create_publisher(
            Float64, '/mono_py_driver/timestep_msg', 10)
        
        self.get_logger().info('GazeboMonoDriver started')

    def img_callback(self, msg):
        # Пробрасываем изображение
        self.pub_img.publish(msg)
        
        # Публикуем timestamp
        ts = Float64()
        ts.data = self.get_clock().now().nanoseconds / 1e9
        self.pub_ts.publish(ts)

def main():
    rclpy.init()
    rclpy.spin(GazeboMonoDriver())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
