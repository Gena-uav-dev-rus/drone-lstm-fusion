
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

### MILESTONE: /vio/odometry publisher работает ✅
- Добавлен nav_msgs::msg::Odometry publisher в MonocularMode
- Публикуется только когда state==2 (OK), не публикует мусор при потере трекинга
- Топик: /vio/odometry, ~11 Hz
- Фикс бага: settingsFilePath накапливался при повторных handshake звонках
  → добавлена baseSettingsFilePath, путь строится заново каждый раз
- nav_msgs добавлен в package.xml (build_depend/exec_depend) и CMakeLists.txt
  (THIS_PACKAGE_INCLUDE_DEPENDS и find_package)

### Этап 1 — итог
- ORB-SLAM3 v1.0 ✅
- ros2_orb_slam3 ROS2 wrapper ✅
- Камера 45° forward-down в Gazebo ✅
- Baylands мир — достаточно фич для tracking ✅
- VIO одометрия публикуется в /vio/odometry ✅

---

## Этап 2 — Depth Anything v2

### Установка
- Репо: github.com/DepthAnything/Depth-Anything-V2
- ФИКС критичный: numpy конфликт между ROS2/cv_bridge (numpy<2) и opencv-python/gradio (numpy>=2)
  → изолируем Depth Anything в отдельный venv:
  python3 -m venv ~/depth_anything_venv
  source ~/depth_anything_venv/bin/activate
  pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
  pip install "numpy<2" opencv-python matplotlib
- Системный Python должен остаться на numpy<2 для cv_bridge:
  pip3 install "numpy<2" --break-system-packages

### Модель — Metric VKITTI Small (outdoor)
- Веса: huggingface.co/depth-anything/Depth-Anything-V2-Metric-VKITTI-Small
- Путь: ~/Depth-Anything-V2/metric_depth/checkpoints/depth_anything_v2_metric_vkitti_vits.pth
- max_depth=80 (outdoor), encoder='vits' (Small, для real-time)
- Indoor вариант (Hypersim) существует отдельно если понадобится

### Тест успешен ✅
- Кадр с камеры дрона (480x640) → depth map в метрах
- На высоте полёта над baylands: depth 5.3м - 17.2м, разумные значения
- GPU P106-100 используется для inference

---

## Архитектурное решение: Depth Anything — Desktop vs Production

### Проблема
ROS 2 cv_bridge требует numpy<2, а Depth Anything зависимости (opencv-python, gradio)
требуют numpy>=2. Прямой конфликт в одном Python окружении.

### Desktop (этап разработки) — venv костыль
- Используем для быстрой итерации и проверки логики
- python3 -m venv ~/depth_anything_venv (полная изоляция, без --system-site-packages)
- PyTorch CUDA + numpy<2 внутри venv
- ROS 2 нода (rclpy) работает в системном Python отдельно
- Обмен данными между процессами — НЕ финальное решение, только для тестов

### Production (Jetson Orin Nano) — TensorRT, без Python
- ПРАВИЛЬНЫЙ путь для финального деплоя на companion computer
- Конвертация: PyTorch (.pth) → ONNX (.onnx) → TensorRT (.engine)
- C++ ROS 2 нода грузит .engine через TensorRT C++ API
- Нет Python в runtime → нет конфликта numpy/venv вообще
- Референс: github.com/ika-rwth-aachen/ros2-depth-anything-v3-trt
- Тот же паттерн что GlobalFusionNode (чистый C++ ROS 2 узел)

### Почему не "общий venv с системными site-packages"
- python3-opencv из apt отстаёт по версиям, без CUDA-ускорения
- На Jetson JetPack даёт свои привязанные версии CUDA/cuDNN — venv-подход менее предсказуем
- TensorRT C++ — стандартный паттерн в продакшен робототехнике, не костыль

### Этап 2 — checkpoint перед перезагрузкой
- Решение по venv: python3 -m venv ~/depth_anything_venv --system-site-packages
  Работает: cv2 4.13.0, rclpy ok, numpy 1.26.4, torch 2.5.1+cu121 CUDA=True все вместе
- Создан пакет depth_anything_ros2:
  - package.xml, CMakeLists.txt (ament_python_install_package)
  - depth_anything_ros2/depth_node.py — подписка /camera, публикует:
    /depth/altitude (Float32, медиана нижней трети кадра в метрах)
    /depth/image (визуализация colormap INFERNO, каждый 5й кадр)
  - Картинка конвертируется вручную (np.frombuffer) без cv_bridge — избегаем
    лишней зависимости от numpy-версии cv_bridge внутри этой ноды
  - Модель: VKITTI Small, путь хардкоднут ~/Depth-Anything-V2/metric_depth
- colcon build прошёл успешно (чистый Python пакет, без компиляции)
- СЛЕДУЮЩИЙ ШАГ ПОСЛЕ РЕБУТА: запустить ноду и проверить что публикует
  Команда запуска:
    source ~/drone-lstm-fusion/ros2_ws/install/setup.bash
    source ~/depth_anything_venv/bin/activate
    ros2 run depth_anything_ros2 depth_node.py
  (порядок source важен — сначала ROS2, потом venv)
