#!/usr/bin/env python3
"""
DroneController — высокоуровневый API для управления дроном через PX4 OFFBOARD,
аналог Clover navigate()/get_telemetry(), но:
  - работает в ЛОКАЛЬНЫХ координатах (не GPS lat/lon)
  - использует /global_odometry (наш LSTM-адаптивный EKF) как источник текущей
    позиции — продолжает работать даже при деградации/потере GPS, так как
    EKF опирается также на VIO и Depth
  - публикует TrajectorySetpoint напрямую в PX4, без отдельного ROS2 сервиса

ВАЖНО про систему координат: PX4 TrajectorySetpoint использует NED
(North-East-Down) локальную систему координат относительно точки взлёта.
Наш /global_odometry тоже в этой же NED-подобной локальной системе
(см. global_fusion_node.cpp), поэтому конвертация не нужна.
"""
import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # PX4 требует QoS BEST_EFFORT для большинства топиков
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.pub_trajectory = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.pub_vehicle_command = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)

        self.sub_status = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self._status_callback, px4_qos)
        self.sub_odom = self.create_subscription(
            Odometry, '/global_odometry', self._odom_callback, 10)

        self.current_position = None  # (x, y, z) в локальной NED системе
        self.current_yaw = 0.0
        self.vehicle_status = None

        # Текущая цель, к которой постоянно публикуется setpoint
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_z = 0.0
        self._target_yaw = 0.0
        self._lock = threading.Lock()

        # Постоянный поток setpoints на 10Hz — ОБЯЗАТЕЛЬНО для OFFBOARD режима PX4
        self._setpoint_timer = self.create_timer(0.1, self._publish_setpoint_loop)

        self.get_logger().info('DroneController initialized, waiting for /global_odometry...')

    def _odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.current_position = (p.x, p.y, p.z)

        # yaw из quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def _publish_setpoint_loop(self):
        # OffboardControlMode — говорим PX4 что управляем по позиции
        mode_msg = OffboardControlMode()
        mode_msg.position = True
        mode_msg.velocity = False
        mode_msg.acceleration = False
        mode_msg.attitude = False
        mode_msg.body_rate = False
        mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_offboard_mode.publish(mode_msg)

        with self._lock:
            x, y, z, yaw = self._target_x, self._target_y, self._target_z, self._target_yaw

        traj_msg = TrajectorySetpoint()
        traj_msg.position = [x, y, z]
        traj_msg.yaw = yaw
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_trajectory.publish(traj_msg)

    def set_target(self, x, y, z, yaw=None):
        """Обновляет текущую цель — публикуется непрерывно фоновым таймером."""
        with self._lock:
            self._target_x = x
            self._target_y = y
            self._target_z = z
            if yaw is not None:
                self._target_yaw = yaw

    def wait_for_odometry(self, timeout=10.0):
        """Блокирующее ожидание первого валидного /global_odometry сообщения."""
        start = time.time()
        while self.current_position is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                return False
        return True

    def arm(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0  # arm
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_vehicle_command.publish(cmd)
        self.get_logger().info('Arm command sent')

    def set_offboard_mode(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0   # custom mode enabled
        cmd.param2 = 6.0   # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_vehicle_command.publish(cmd)
        self.get_logger().info('Offboard mode command sent')

    def land(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_vehicle_command.publish(cmd)
        self.get_logger().info('Land command sent')

    def fly_to(self, x, y, z, yaw=None, tolerance=0.3, timeout=60.0):
        """
        Летит к точке (x,y,z) в локальной NED системе координат
        (относительно точки взлёта, как видит её /global_odometry).
        Блокирует выполнение пока не достигнет точки или не истечёт timeout.
        """
        self.set_target(x, y, z, yaw)
        start = time.time()

        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.current_position is None:
                continue

            cx, cy, cz = self.current_position
            dist = math.sqrt((cx - x)**2 + (cy - y)**2 + (cz - z)**2)

            if dist < tolerance:
                self.get_logger().info(f'Reached target ({x:.2f}, {y:.2f}, {z:.2f})')
                return True

        self.get_logger().warn(f'Timeout reaching target ({x:.2f}, {y:.2f}, {z:.2f})')
        return False

    def get_position(self):
        """Текущая позиция (x, y, z) в локальной NED системе из /global_odometry."""
        return self.current_position

    def get_yaw(self):
        return self.current_yaw
