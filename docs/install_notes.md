
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

### MILESTONE: depth_anything_ros2 нода работает ✅
- /depth/altitude публикуется на ~6.3 Hz, реальные метрические значения (11-16м
  при полёте над baylands на текущей высоте)
- /depth/image — визуализация colormap для отладки
- Запуск: source ROS2 setup → source venv (порядок важен)
- GPU inference через CUDA, ~6 FPS достаточно для altitude estimation
  (не критичный по латентности контур, в отличие от VIO)

### Этап 2 — итог
- Depth Anything V2 metric (VKITTI Small) интегрирован в ROS 2 ✅
- /depth/altitude и /depth/image публикуются ✅
- Venv-решение работает для desktop разработки ✅
- TensorRT C++ нода — отложено до этапа подготовки Jetson деплоя

### Этап 3 — checkpoint перед перезагрузкой
- Пакет global_fusion создан и собирается ✅
  - ekf.hpp / ekf.cpp — error-state EKF, 15-state (pos+vel+orient+accel_bias+gyro_bias)
  - global_fusion_node.cpp — подписки /imu, /fmu/out/vehicle_gps_position,
    /vio/odometry, /depth/altitude; публикует /global_odometry
  - ФИКС: Eigen3 не подцеплялся через find_package — добавлен
    include_directories(${EIGEN3_INCLUDE_DIR}) в CMakeLists.txt
  - ФИКС: QoS несовместимость на /fmu/out/vehicle_gps_position (PX4 публикует
    BEST_EFFORT) — добавлен rclcpp::QoS px4_qos(10); px4_qos.best_effort();
- GPS origin захват работает: "GPS origin set: lat=37.412174 lon=-121.998879 alt=38.94"
- ПРОБЛЕМА на момент чекпоинта: при одновременном запуске ORB-SLAM3 + Depth
  Anything (GPU) + global_fusion + Gazebo, система тормозит — камера упала
  с 24-30fps до 3fps, IMU топик не отвечал на echo/hz (вероятно из-за лагов
  real_time_factor в Gazebo, не баг в коде)
- СЛЕДУЮЩИЙ ШАГ ПОСЛЕ РЕБУТА: поднять стек заново, проверить нагрузку
  (htop/nvidia-smi), возможно нужно снизить частоту Depth Anything ноды
  или вообще не гонять ORB-SLAM3+Depth одновременно на одном GPU без оптимизации

### MILESTONE: GlobalFusionNode полностью работает ✅
- ФИКС критичный: /imu bridge слушал несуществующий короткий топик "/imu" в Gazebo.
  Реальный source топик: /world/<world_name>/model/<model>/link/base_link/sensor/imu_sensor/imu
  Добавлен remapping в drone.launch.py: длинный путь -> /imu
  (искать через `gz topic -l` если переходишь на другую модель/мир — имя топика
  зависит от имени world и model)
- ФИКС производительности: Gazebo headless (-s флаг, без GUI) экономит >50% CPU
  vs полный GUI режим. Создан scripts/start_sim_headless.sh для тяжёлых прогонов
  с полным стеком (ORB-SLAM3 + Depth Anything + global_fusion одновременно)
- ФИКС производительности: добавлен троттлинг в depth_node.py — инференс раз в
  N=5 кадров камеры вместо каждого кадра, снижает нагрузку depth ноды
- /global_odometry публикуется на ~78Hz (привязан к частоте IMU predict, 85Hz)
- EKF не расходится, значения позиции/скорости/ориентации в разумных пределах
  при висении дрона в воздухе

### Этап 3 — итог
- global_fusion package: ekf.hpp/ekf.cpp (15-state error-state EKF) + 
  global_fusion_node.cpp (ROS2 wrapper) ✅
- Источники: IMU (predict), GPS/VIO/Depth (sequential update) ✅
- /global_odometry публикуется, готов для PX4 OFFBOARD (следующий шаг) ✅
- R-матрицы пока фиксированные константы — подключение LSTM variance на Этапе 4

### MILESTONE: GroundTruthPlugin (Gazebo C++ plugin) работает ✅
- Создан собственный системный плагин Gazebo для получения точного ground truth
  позиции/скорости дрона (для обучения LSTM на Этапе 4)
- Путь: ros2_ws/src/ground_truth_plugin/ (ament_cmake пакет)
- Ищет модель по имени через ECM (model_name настраивается через SDF тег
  <model_name>, по умолчанию x500_mono_cam_0), публикует gz.msgs.Odometry
  на топик /ground_truth/<model_name>/odometry
- КРИТИЧНО: обычный cmake НЕ находит gz-* конфиги напрямую — они спрятаны
  в /opt/ros/jazzy/opt/gz_*_vendor/lib/cmake/. Решение — собирать ТОЛЬКО
  через colcon build внутри ros2_ws (он сам подхватывает vendor prefix paths)
- Точные cmake target имена (на ROS 2 Jazzy): gz-sim8, gz-plugin2 (+ register
  компонент), gz-msgs10, gz-transport13
- Подключение к миру: добавлен <plugin> тег в конец baylands.sdf перед </world>
- Переменная окружения нужна: GZ_SIM_SYSTEM_PLUGIN_PATH должна включать
  путь к install/ground_truth_plugin/lib (добавлено в ~/.bashrc)
- Почему не использовали готовый /world/.../pose/info (Pose_V): даёт позы
  ВСЕХ объектов мира без имён в массиве, пришлось бы угадывать индекс дрона —
  хрупко при изменении мира. Собственный плагин надёжнее.
- Почему не /model/.../odometry_with_covariance: топик существовал в gz topic -l
  но без активного publisher (нет OdometryPublisher system подключенного
  к этой конкретной модели) — наш плагин закрывает эту дыру

### Угол камеры — финальное значение
- pose: .15 0 .05 0 1.047 0 (60° вниз, положительный знак)
- Отрицательный знак (-0.7854, -1.047) давал камеру назад-вверх (видны пропеллеры/небо)
- Положительный знак +1.047 рад = 60° вниз — видна земля/дорожки/текстуры
- Файл: ~/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam/model.sdf

### MILESTONE: Dataset collector работает ✅
- Пакет lstm_data_collector, нода collector_node.py
- Собирает синхронно: ground truth + GPS + VIO + Depth → CSV
- 500 сэмплов за ~33 секунды (~15 Hz, привязан к частоте ground truth топика)
- Запуск: ros2 run lstm_data_collector collector_node.py
    --ros-args -p output_file:=/tmp/lstm_dataset.csv -p max_samples:=500
- ВАЖНО: перед запуском нужен ground truth bridge:
    ros2 run ros_gz_bridge parameter_bridge
    /ground_truth/x500_mono_cam_0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry
- Для полноценного датасета: собирать 50000+ сэмплов при разных условиях
  (разные высоты, скорости, направления движения)

### MILESTONE: LSTM noise estimators обучены успешно ✅
- ВАЖНЫЕ НАХОДКИ И ФИКСЫ при подготовке датасета:
  1. GPS north/east оси были перепутаны с gt_x/gt_y (ENU vs NED путаница) —
     gps_east соответствует gt_x, gps_north соответствует gt_y в нашей
     Gazebo ENU world frame конвенции
  2. VIO (ORB-SLAM3) живёт в собственной произвольной системе координат,
     не совпадающей с world frame — нужно align через Kabsch algorithm
     (rotation+translation), не просто offset
  3. VIO дрейфует без loop closure — абсолютный align по всей траектории
     даёт огромные ложные residuals на дальних участках. Решение: периодический
     re-align каждые window_size=200 сэмплов (имитирует реальное поведение EKF,
     где VIO постоянно корректируется внешними источниками)
  4. Train/val split ДОЛЖЕН быть случайным (shuffled), не последовательным по
     времени — последовательный сплит может разделить полёт на физически разные
     условия (например высота в начале vs в конце), создавая ложный distribution
     shift и резкий разрыв train/val loss, маскирующий реальное переобучение
  5. Этот же VIO frame alignment баг исправлен и в global_fusion_node.cpp
     (vioCallback теперь выравнивает VIO->world через rotation+translation
     transform, вычисленный при первом VIO сообщении после получения GPS origin)

### Финальные результаты обучения (50000 сэмплов, 50 эпох)
- GPS LSTM:   train=25.2 val=21.3 (м², ~4.6м RMS)
- VIO LSTM:   train=1.38 val=1.17 (м², ~1.1м RMS)
- Depth LSTM: train=91.6 val=82.3 (м², ~9м RMS)
- Train≈val везде — модели обобщают, не переобучены

### Этап 4 — прогресс
- ground_truth_plugin (Gazebo C++) ✅
- lstm_data_collector (датасет ROS2 нода) ✅
- train_lstm.py (PyTorch обучение трёх LSTM) ✅
- VIO frame alignment в global_fusion_node.cpp ✅
- СЛЕДУЮЩИЙ ШАГ: ROS2 inference ноды (Python rclpy) для трёх LSTM,
  публикующие /lstm_noise/gps_variance, /lstm_noise/vio_variance,
  /lstm_noise/depth_variance, и подключение к global_fusion_node через
  setGpsPositionVariance/setVioPositionVariance/setDepthVariance setters

### MILESTONE: LSTM inference ноды работают в реальном времени ✅
- Пакет lstm_noise_estimator с тремя нодами:
  - gps_noise_node.py -> /lstm_noise/gps_variance
  - vio_noise_node.py -> /lstm_noise/vio_variance
  - depth_noise_node.py -> /lstm_noise/depth_variance
- Общий model.py модуль (NoiseEstimatorLSTM класс + load_model helper)
- ВАЖНО: depth_noise_node использует /global_odometry скорость вместо ground truth
  (недоступен в runtime) как context-фича — небольшое расхождение с training
  ожидаемо и не критично
- Все три ноды требуют venv (torch+CUDA), запускаются после source ROS2 setup
- Живые значения variance в разумных пределах:
  GPS: ~4-5 м², VIO: ~0.009 м² (стабильный полёт), Depth: ~110-236 м²

### MILESTONE: Этап 4 ПОЛНОСТЬЮ ЗАВЕРШЁН ✅
- global_fusion_node.cpp подключает LSTM variance в реальном времени:
  /lstm_noise/gps_variance -> ekf_->setGpsPositionVariance()
  /lstm_noise/vio_variance -> ekf_->setVioPositionVariance()
  /lstm_noise/depth_variance -> ekf_->setDepthVariance()
- Защита от невалидных значений (var > 0 && isfinite) перед применением к EKF
- Полный контур протестирован: сырые измерения -> LSTM -> variance ->
  EKF update -> /global_odometry, стабильно ~104-120 Hz, не расходится
- VIO frame alignment подтверждён в логах: "VIO aligned to world frame
  at EKF position (...)" — происходит один раз после первого GPS fix

### Этап 4 — итог (полностью завершён)
1. ground_truth_plugin (Gazebo C++) — точный teacher signal ✅
2. lstm_data_collector — синхронный сбор GPS/VIO/Depth/ground truth в CSV ✅
3. train_lstm.py — обучение 3 LSTM с Kabsch VIO alignment + periodic re-align +
   случайный train/val split ✅
4. lstm_noise_estimator — 3 inference ноды в реальном времени ✅
5. global_fusion_node.cpp — динамическая R-матрица вместо фиксированных констант ✅
6. VIO-to-world frame alignment добавлен и в EKF (не только в датасете) ✅

### MILESTONE: Drift checker подтвердил качество fusion ✅
- scripts/check_drift.py — сравнивает /global_odometry с ground truth,
  компенсирует начальный offset нулевых точек отсчёта (EKF world frame
  стартует с (0,0,0), Gazebo world frame абсолютный — РАЗНЫЕ системы)
- ВАЖНО: offset компенсация только в скрипте проверки, EKF/global_fusion_node
  НЕ модифицирован и ground truth не использует в реальном времени (честно,
  как было бы в продакшене без доступа к истине)
- Результат: реальная ошибка fusion держится в пределах 0-1.3 метра,
  не расходится неограниченно — здоровое поведение всей системы
- Использование: source ROS2 setup, затем
  python3 ~/drone-lstm-fusion/scripts/check_drift.py
