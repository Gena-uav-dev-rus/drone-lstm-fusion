# Drone LSTM Fusion — Installation Notes

## Hardware
- HP Pavilion dv6, i5 3rd gen, 8GB RAM, 124GB SSD
- Intel HD Graphics (CPU-only, no CUDA)
- Нативный Ubuntu 24.04 LTS

## Ограничения железа
- Нет CUDA — SuperPoint и Depth Anything на CPU (медленно)
- LSTM обучение — Google Colab
- Gazebo запускать с HEADLESS=1 при работе через SSH

---

## Этап 0 — Базовая установка

### ROS 2 Jazzy
- Стандартная установка через apt
- Проверка: ros2 run demo_nodes_cpp talker + listener

### Gazebo Harmonic 8.11.0
- ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim

### PX4 v1.15 SITL
- ФИКС: Ubuntu 24.04 блокирует pip
  sed -i 's/pip install/pip install --break-system-packages/g' Tools/setup/ubuntu.sh
  sed -i 's/pip3 install/pip3 install --break-system-packages/g' Tools/setup/ubuntu.sh
- openjdk-14 не найден — не критично
- Запуск: HEADLESS=1 make px4_sitl gz_x500

### Micro XRCE-DDS Agent
- v2.4.2 не собирается (FastDDS 2.12.x проблема) — используем v2.4.3
- cmake -DEPROSIMA_BUILD_TESTS=OFF

---

## Этап 1 — ORB-SLAM3

### Pangolin v0.8
- ФИКС GCC 13: массовый патч cstdint
  find . -name "*.cpp" -o -name "*.h" | xargs grep -l "uint32_t\|uint16_t\|uint8_t" | xargs sed -i '1s/^/#include <cstdint>\n/'
- ФИКС ffmpeg: -DBUILD_PANGOLIN_FFMPEG=OFF

### ORB-SLAM3 v1.0
- ФИКС sigslot: cmake -DCMAKE_CXX_STANDARD=14
- RealSense не нужен — используем Gazebo камеру

### ros2_orb_slam3
- jazzy ветка: git clone --branch jazzy
- colcon build --packages-select ros2_orb_slam3
