# Drone LSTM Fusion — Installation Notes
# Реальный опыт установки с граблями и фиксами

---

## Hardware
- HP Pavilion dv6, i5 3rd gen, 8GB RAM, 124GB SSD
- Intel HD Graphics (CPU-only, no CUDA)
- Нативный Ubuntu 24.04 LTS

## Ограничения железа
- Нет CUDA → SuperPoint и Depth Anything работают на CPU (медленно)
- LSTM обучение → Google Colab
- Gazebo запускать с HEADLESS=1 (нет дисплея при работе через SSH)

---

## Этап 0 — Базовая установка

### 0.1 Ubuntu 24.04
- Флешка писалась через balenaEtcher (Rufus давал ошибки с MBR/GPT на старом BIOS)
- dv6 — Legacy BIOS, GPT не поддерживается при загрузке

### 0.2 ROS 2 Jazzy
```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop-full
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
✅ Проверка: `ros2 run demo_nodes_cpp talker` + `ros2 run demo_nodes_py listener`

### 0.3 Gazebo Harmonic
```bash
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim
```
✅ Проверка: `gz sim --version` → Gazebo Sim 8.11.0

### 0.4 PX4 v1.15 SITL
```bash
# Зависимости
sudo apt install -y ninja-build exiftool astyle python3-empy python3-toml \
  python3-numpy python3-dev gawk ant openjdk-11-jdk-headless zip unzip

# ВАЖНО: Ubuntu 24.04 блокирует pip без флага
pip3 install --break-system-packages kconfiglib jinja2 jsonschema \
  pyros-genmsg packaging toml numpy empy

git clone https://github.com/PX4/PX4-Autopilot.git --recursive --branch v1.15.0
cd PX4-Autopilot

# ФИКС: патчим скрипт установки под Ubuntu 24.04
sed -i 's/pip install/pip install --break-system-packages/g' Tools/setup/ubuntu.sh
sed -i 's/pip3 install/pip3 install --break-system-packages/g' Tools/setup/ubuntu.sh
bash Tools/setup/ubuntu.sh --no-nuttx
# openjdk-14 не найден — не критично, игнорируем
```
✅ Проверка: `HEADLESS=1 make px4_sitl gz_x500` → "Ready for takeoff!"
⚠️ Всегда запускать с HEADLESS=1 — нет дисплея при SSH

### 0.5 Micro XRCE-DDS Agent v2.4.3
```bash
# v2.4.2 не собирается — проблема с FastDDS 2.12.x
# Используем v2.4.3
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git --branch v2.4.3
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DEPROSIMA_BUILD_TESTS=OFF
make -j2
sudo make install
sudo ldconfig /usr/local/lib/
```
✅ Проверка: `MicroXRCEAgent udp4 -p 8888`

### 0.6 Финальная проверка
```bash
# Терминал 1
MicroXRCEAgent udp4 -p 8888
# Терминал 2
cd ~/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_x500
# Терминал 3
ros2 topic list | grep fmu
```
✅ Видны топики /fmu/out/vehicle_odometry, /fmu/in/trajectory_setpoint и др.

---

## Этап 1 — ORB-SLAM3

### 1.1 Pangolin v0.8
```bash
git clone https://github.com/stevenlovegrove/Pangolin.git --branch v0.8
cd Pangolin

# ФИКС: GCC 13 требует явный #include <cstdint> везде
# Массовый патч всех проблемных файлов:
find . -name "*.cpp" -o -name "*.h" | xargs grep -l "uint32_t\|uint16_t\|uint8_t" | \
  xargs sed -i '1s/^/#include <cstdint>\n/'

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_PANGOLIN_FFMPEG=OFF  # ФИКС: ffmpeg константы устарели в новых версиях
make -j2
sudo make install
sudo ldconfig
```
✅ Проверка: `find /usr/local/lib -name "*.so" | grep pango` → список .so файлов

### 1.2 ORB-SLAM3 v1.0
```bash
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git --branch v1.0-release
cd ORB_SLAM3
mkdir -p build && cd build
# ФИКС: нужен C++14 для sigslot из Pangolin
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14
make -j2
```
✅ Проверка: `ls ~/ORB_SLAM3/lib/libORB_SLAM3.so`
⚠️ RealSense SDK не найден — не критично, используем Gazebo камеру

### 1.3 ros2_orb_slam3 (jazzy ветка)
```bash
cd ~/drone-lstm-fusion/ros2_ws/src
git clone https://github.com/Mechazo11/ros2_orb_slam3.git --branch jazzy
cd ~/drone-lstm-fusion/ros2_ws
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ros2_orb_slam3 --cmake-args \
  -DORB_SLAM3_DIR=/home/sqwaer/ORB_SLAM3 \
  -DCMAKE_BUILD_TYPE=Release
```

