#!/usr/bin/env python3
"""
Полётная миссия для дрона на PX4 OFFBOARD / ROS 2 / симуляция Gazebo.

Портировано с ROS 1 / COEX Clover скрипта (start.py) на ROS 2 / PX4.
Главное отличие архитектуры: вместо Clover navigate()/get_telemetry() сервисов
используется DroneController — публикует TrajectorySetpoint напрямую в PX4,
координаты ЛОКАЛЬНЫЕ (NED от точки взлёта), источник позиции — /global_odometry
(наш LSTM-адаптивный EKF, устойчивый к деградации/потере GPS).

УБРАНО из оригинала (специфично для реального Clover-дрона, не для симуляции):
  - Wi-Fi сканирование для точной посадки (adjust_height_and_land)
  - start_selfcheck() — Clover-специфичная диагностика
  - Реальный Arduino сервопривод — оставлена ЗАГЛУШКА (ServoController),
    готовая для подключения реального железа позже

СОХРАНЕНО: расчёт баллистики сброса груза (calculate_drop_point), структура
полёта (взлёт -> поворот -> полёт по дистанции -> [сброс] -> возврат -> посадка).
"""
import math
import time

import rclpy

from drone_mission.drone_controller import DroneController


# ─── Заглушка сервопривода (готова для реального железа на Jetson/дроне) ──────

class ServoController:
    """
    ЗАГЛУШКА. На реальном дроне здесь будет тот же Arduino-based код что
    в оригинальном start.py (find_arduino, serial communication).
    Для симуляции просто логирует действия и имитирует задержку.
    """
    def __init__(self, logger):
        self.logger = logger
        self.servo_available = False
        self.logger.info('ServoController: заглушка (реальный сервопривод не подключен)')

    def drop_potato(self, delay=1.0):
        self.logger.info('ServoController: имитация сброса груза...')
        time.sleep(delay)
        self.logger.info('ServoController: сброс (имитация) выполнен')
        return False  # False = имитация, не реальный сброс

    def cleanup(self):
        pass


# ─── Баллистика сброса (без изменений из оригинала) ───────────────────────────

def calculate_drop_point(distance, height, speed, potato_weight, logger):
    """
    Рассчитывает точку сброса груза с учётом сопротивления воздуха,
    чтобы груз приземлился в целевой точке на дистанции distance.
    """
    g = 9.81
    mass = potato_weight / 1000.0
    potato_diameter = (mass / 1050) ** (1 / 3)
    air_resistance = 0.47
    area = math.pi * (potato_diameter / 2) ** 2
    air_density = 1.225
    v_terminal = math.sqrt((2 * mass * g) / (air_density * area * air_resistance))
    fall_time = height / v_terminal * (1 - math.exp(-v_terminal * height / (2 * height)))
    horizontal_distance = speed * fall_time
    drop_point = distance - horizontal_distance

    logger.info("РАСЧЕТ ТОЧКИ СБРОСА:")
    logger.info(f"  Время падения: {fall_time:.2f} сек")
    logger.info(f"  Терминальная скорость: {v_terminal:.2f} м/с")
    logger.info(f"  Горизонтальное смещение: {horizontal_distance:.2f} м")
    logger.info(f"  Точка сброса: {drop_point:.2f} м от старта")
    return drop_point


# ─── Этапы миссии ──────────────────────────────────────────────────────────────

def takeoff(drone: DroneController, target_height=10.0):
    """
    Взлёт: arm -> offboard mode -> подъём на target_height метров.
    target_height в метрах (положительное число), внутри конвертируется
    в NED (отрицательный Z = вверх).
    """
    if not drone.wait_for_odometry(timeout=10.0):
        drone.get_logger().error('Нет данных одометрии, взлёт невозможен')
        return False

    start_x, start_y, start_z = drone.get_position()
    drone.get_logger().info(f'Стартовая позиция: ({start_x:.2f}, {start_y:.2f}, {start_z:.2f})')

    # Прогреваем поток setpoints перед переключением в OFFBOARD (PX4 требование)
    drone.set_target(start_x, start_y, start_z)
    for _ in range(20):
        rclpy.spin_once(drone, timeout_sec=0.1)

    drone.set_offboard_mode()
    time.sleep(0.5)
    drone.arm()
    time.sleep(1.0)

    target_z = start_z - target_height  # NED: вверх = отрицательный Z
    drone.get_logger().info(f'Взлёт на высоту {target_height} м (целевой Z={target_z:.2f})')

    # Таймаут пропорционален высоте — SITL обычно набирает ~1 м/с по умолчанию
    climb_timeout = max(30.0, target_height * 4.0)
    success = drone.fly_to(start_x, start_y, target_z, tolerance=0.5, timeout=climb_timeout)
    if success:
        drone.get_logger().info('Взлёт выполнен успешно')
    else:
        drone.get_logger().warn('Не удалось достичь целевой высоты взлёта')
    return success


def turn_to_yaw(drone: DroneController, target_yaw_deg, tolerance_deg=3.0, timeout=20.0):
    """Поворот на месте к целевому курсу (yaw в градусах, 0=North, по часовой)."""
    pos = drone.get_position()
    if pos is None:
        return False
    x, y, z = pos
    target_yaw_rad = math.radians(target_yaw_deg)

    drone.get_logger().info(f'Поворот к курсу {target_yaw_deg}°')
    success = drone.fly_to(x, y, z, yaw=target_yaw_rad, tolerance=999.0, timeout=0.1)
    # fly_to с tolerance=999 сразу "успешен" по позиции, реальная проверка — yaw ниже

    start = time.time()
    while time.time() - start < timeout:
        rclpy.spin_once(drone, timeout_sec=0.1)
        current_yaw_deg = math.degrees(drone.get_yaw())
        diff = abs(((target_yaw_deg - current_yaw_deg + 180) % 360) - 180)
        if diff < tolerance_deg:
            drone.get_logger().info(f'Курс достигнут: {current_yaw_deg:.1f}°')
            return True

    drone.get_logger().warn('Таймаут поворота к целевому курсу')
    return False


def fly_distance(drone: DroneController, distance, target_height, speed=5.0):
    """
    Полёт вперёд (по текущему курсу) на distance метров, удерживая target_height.
    Возвращает (success, actual_distance).
    """
    pos = drone.get_position()
    if pos is None:
        return False, distance

    start_x, start_y, start_z = pos
    yaw = drone.get_yaw()

    target_x = start_x + distance * math.cos(yaw)
    target_y = start_y + distance * math.sin(yaw)
    target_z = start_z

    drone.get_logger().info(
        f'Полёт на {distance:.1f} м по курсу {math.degrees(yaw):.1f}° '
        f'(цель: {target_x:.2f}, {target_y:.2f}, {target_z:.2f})')

    timeout = max(30.0, distance / max(speed, 0.5) * 2)
    success = drone.fly_to(target_x, target_y, target_z, tolerance=1.0, timeout=timeout)
    return success, distance


def return_home(drone: DroneController, home_x, home_y, home_z, timeout=60.0):
    """Возврат в точку взлёта."""
    drone.get_logger().info(f'Возврат в точку взлёта ({home_x:.2f}, {home_y:.2f}, {home_z:.2f})')
    return drone.fly_to(home_x, home_y, home_z, tolerance=1.0, timeout=timeout)


def land_and_disarm(drone: DroneController):
    drone.get_logger().info('Посадка...')
    drone.land()
    time.sleep(8.0)
    drone.get_logger().info('Посадка выполнена (предположительно)')
    return True


def countdown_timer(seconds, logger):
    try:
        seconds = int(seconds)
        if seconds < 1 or seconds > 999:
            raise ValueError("Время должно быть от 1 до 999 секунд")
        logger.info("НАЧАЛО ОБРАТНОГО ОТСЧЕТА")
        while seconds > 0:
            print(f'\rСтарт через: {seconds:3d} сек', end='', flush=True)
            time.sleep(1)
            seconds -= 1
        print()
        logger.info("ВЗЛЕТ!")
    except ValueError as e:
        logger.error(f"Ошибка в таймере: {e}")
        return False
    except KeyboardInterrupt:
        logger.info("Отсчёт прерван")
        return False
    return True


# ─── Главная функция миссии ────────────────────────────────────────────────────

def main():
    rclpy.init()
    drone = DroneController()
    logger = drone.get_logger()
    servo = ServoController(logger)

    try:
        logger.info("=" * 50)
        logger.info("Параметры миссии (интерактивный ввод)")

        height_input = input("Высота полета (м) от 5 до 50: ")
        azimuth_input = input("Азимут куда летим (0-359): ")
        distance_input = input("Дальность полета (м): ")
        speed_input = input("Скорость полета (м/с) от 0.5 до 10.0: ")
        timer_input = input("Время ожидания перед взлетом (1-999 сек): ")

        while True:
            drop_mode = input("Нужно сделать сброс? (да/нет): ").lower()
            if drop_mode in ['да', 'нет']:
                break
            print("Введите 'да' или 'нет'")

        height = float(height_input)
        azimuth = float(azimuth_input)
        distance = float(distance_input)
        speed = float(speed_input)
        timer_seconds = int(timer_input)

        if not (5 <= height <= 50):
            raise ValueError("Высота должна быть от 5 до 50 м (симуляция)")
        if not (0 <= azimuth < 360):
            raise ValueError("Азимут должен быть от 0 до 360")
        if distance <= 0:
            raise ValueError("Дистанция должна быть больше 0")
        if not (0.5 <= speed <= 10.0):
            raise ValueError("Скорость должна быть от 0.5 до 10.0 м/с")

        logger.info("=" * 50)
        logger.info(f"Высота: {height} м, Азимут: {azimuth}°, "
                    f"Дистанция: {distance} м, Скорость: {speed} м/с")
        logger.info(f"Режим сброса: {'Включен' if drop_mode == 'да' else 'Выключен'}")
        logger.info("=" * 50)

        confirm = input("Подтвердите старт миссии (да/нет): ").lower()
        if confirm != 'да':
            logger.info("Миссия отменена")
            return

        if not countdown_timer(timer_seconds, logger):
            logger.info("Миссия отменена")
            return

        mission_success = True

        if not takeoff(drone, target_height=height):
            logger.error("Ошибка взлёта!")
            land_and_disarm(drone)
            return

        home_x, home_y, home_z = drone.get_position()

        if not turn_to_yaw(drone, azimuth):
            logger.error("Не удалось выполнить поворот")
            land_and_disarm(drone)
            return

        if drop_mode == 'да':
            drop_distance = calculate_drop_point(distance, height, speed, potato_weight=500, logger=logger)
            logger.info("Выполняю полёт до точки сброса...")
            success, actual_distance = fly_distance(drone, drop_distance, height, speed)
            if success:
                logger.info("Стабилизация перед сбросом...")
                time.sleep(1)
                servo.drop_potato(delay=1.2)
                logger.info("Долёт до конечной точки...")
                fly_distance(drone, distance - drop_distance, height, speed)
            else:
                mission_success = False
                logger.warn("Не удалось достичь точки сброса!")
        else:
            logger.info("Выполняю полёт до конечной точки...")
            success, actual_distance = fly_distance(drone, distance, height, speed)
            if not success:
                mission_success = False
                logger.warn("Не удалось достичь конечной точки!")

        logger.info("Возвращение в точку старта...")
        if not return_home(drone, home_x, home_y, home_z):
            mission_success = False
            logger.warn("Не удалось вернуться домой!")

        land_and_disarm(drone)

        logger.info("Миссия завершена" + (" успешно!" if mission_success else " с ошибками!"))

    except (ValueError, KeyboardInterrupt) as e:
        logger.error(f"Миссия прервана: {e}")
        try:
            land_and_disarm(drone)
        except Exception:
            logger.error("Ошибка при аварийной посадке!")
    except Exception as e:
        logger.error(f"Ошибка выполнения миссии: {e}")
        try:
            land_and_disarm(drone)
        except Exception:
            logger.error("Ошибка при аварийной посадке!")
    finally:
        servo.cleanup()
        drone.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
