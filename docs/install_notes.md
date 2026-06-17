
### ros2_orb_slam3 — дополнения
- ФИКС OOM: 8GB RAM не хватает для сборки
  sudo fallocate -l 4G /swapfile
  sudo chmod 600 /swapfile
  sudo mkswap /swapfile
  sudo swapon /swapfile
  echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
- ФИКС warnings-as-errors: добавить -DCMAKE_CXX_FLAGS="-w"
- Собирать в один поток: MAKEFLAGS="-j1" colcon build
- Время сборки: ~10 минут с swap, ~1.5 часа без swap (OOM kill)

### Gazebo камера → ROS 2
- Используем x500_mono_cam модель (x500 + mono_cam спереди)
- Запуск: HEADLESS=1 make px4_sitl gz_x500_mono_cam
- ВАЖНО: нужен MicroXRCEAgent запущен параллельно
- Bridge для камеры:
  ros2 run ros_gz_bridge parameter_bridge \
    /camera@sensor_msgs/msg/Image@gz.msgs.Image \
    /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
- Камера даёт ~24 FPS на i5 3rd gen

### Запуск стека — порядок важен
1. MicroXRCEAgent udp4 -p 8888
2. Xvfb :99 -screen 0 1280x720x24 & export DISPLAY=:99
3. HEADLESS=1 make px4_sitl gz_x500_mono_cam
4. ros2 launch drone_bringup drone.launch.py
5. ros2 run ros2_orb_slam3 mono_node_cpp ...

### PX4 армирование в симуляторе
- ФИКС degenerate accel: param set ATT_EN 0, EKF2_EN 1
- ФИКС компас: param set SYS_HAS_MAG 0, EKF2_MAG_TYPE 5
- ФИКС flight termination: param set CBRK_FLIGHTTERM 121212
- ФИКС GPS: param set EKF2_GPS_CTRL 0, COM_ARM_WO_GPS 1
- Армирование: commander arm -f

### ORB-SLAM3 фиксы
- enablePangolinWindow = false (нет GPU)
- enableOpenCVWindow = false (нет дисплея при SSH)
- Добавить проверку if(!pAgent) return; в Img_callback
- packagePath = "drone-lstm-fusion/ros2_ws/src/ros2_orb_slam3/"

### QGroundControl
- wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.3.0/QGroundControl.AppImage
- export DISPLAY=:99 перед запуском

---

## Target Hardware

### Desktop (симуляция и разработка)
- CPU: 4+ ядра x86
- RAM: 16GB рекомендуется (8GB минимум)
- GPU: NVIDIA GTX 1060+ 6GB VRAM
- SSD: 100GB+
- OS: Ubuntu 24.04 LTS

### Companion Computer (на дроне)
- Jetson Orin Nano 8GB
- CUDA 1024 cores
- Вес: 45г
- OS: JetPack 6.x (Ubuntu 22.04 base)

## Docker образы (план)

### drone-sim (Desktop)
- Gazebo Harmonic + PX4 SITL
- ROS 2 Jazzy full
- ORB-SLAM3 + SuperPoint
- Depth Anything v2
- LSTM training (PyTorch)
- RViz2 + Foxglove

### drone-companion (Jetson Orin Nano)
- ROS 2 Jazzy headless
- ORB-SLAM3 + SuperPoint (CUDA)
- Depth Anything v2 Small (TensorRT)
- LSTM inference (TorchScript)
- GlobalFusionNode (C++ EKF)
- Offboard controller

### Desktop — запуск стека
- DISPLAY=:0 обязательно перед запуском
- Газебо открывается с GUI через Anydesk
- QGroundControl: sudo usermod -a -G dialout sqwaer
- COM_RC_IN_MODE=1 для виртуального джойстика
- GPS fix нужен для стабильного полёта

### Успешный тест
- Дрон взлетел, держал высоту, сел по команде QGC ✅

### Desktop PC — полный стек работает
- Gazebo запускается с GUI через DISPLAY=:0
- QGroundControl работает, дрон летает
- ORB-SLAM3 запускается, State=1 (NOT_INITIALIZED)

### Проблема инициализации ORB-SLAM3
- Камера горизонтальная → видит только небо и землю (нет фич)
- Повернули камеру вниз: pitch=1.5708 рад
- Камера видит дрон себя (ноги, пропеллеры)
- Нужно: сдвинуть камеру чтобы видела только землю

### Следующий шаг
- Правильно позиционировать камеру (вниз, без корпуса дрона в кадре)
- Добавить текстуру на землю для фич ORB
- Добиться State=2 (OK) — трекинг работает
- Добавить publisher одометрии → /vio/odometry

### Запуск через скрипт
- scripts/start_sim.sh — автозапуск всего стека
- Порядок: Agent → Gazebo (25с) → PX4 (20с) → Bridge (8с) → SLAM (5с) → QGC
- Мир: baylands (богатый, парк + дорожки)
- GZ_SIM_RESOURCE_PATH нужен для загрузки моделей PX4
- Раздельный запуск: gz sim отдельно, потом PX4_GZ_STANDALONE=1

### Проблема ORB-SLAM3 State=1
- Камера 45 градусов вперёд-вниз: pose .15 0 .05 0 -0.7854 0
- Baylands даёт богатую сцену для трекинга
- Нужно проверить State=2 в полёте над городом

### ORB-SLAM3 State: 2 (OK) — ДОСТИГНУТО ✅
- Мир: baylands.sdf (богатый, со зданиями и текстурами)
- Камера: 45° вперёд-вниз, pose=".15 0 .05 0 -0.7854 0"
- Запуск раздельный: Gazebo standalone → PX4 PX4_GZ_STANDALONE=1
- Критично: GZ_SIM_RESOURCE_PATH должен включать models И worlds
- Критично: имя world в sdf файле должно совпадать с PX4_GZ_WORLD

### Полный автозапуск
- Скрипт: scripts/start_sim.sh
- Запускает всё через tmux с правильными задержками
- 6 окон: agent, gazebo, px4, bridge, slam, qgc
