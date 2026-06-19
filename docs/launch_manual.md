# Мануал запуска — Drone LSTM Fusion

## Скрипты автозапуска

### Headless (рекомендуется для полного стека)
```bash
bash ~/drone-lstm-fusion/scripts/start_sim_headless.sh
```

### С GUI Gazebo (для визуальной отладки)
```bash
bash ~/drone-lstm-fusion/scripts/start_sim.sh
```

## Окна tmux (Ctrl+B + цифра для переключения)

| # | Имя | Команда | Ждём |
|---|-----|---------|------|
| 0 | agent | MicroXRCEAgent udp4 -p 8888 | publisher created |
| 1 | gazebo | gz sim [-s] -r baylands.sdf | мир загружен (25-30 сек) |
| 2 | px4 | make px4_sitl gz_x500_mono_cam | Ready for takeoff! |
| 3 | bridge | ros2 launch drone_bringup drone.launch.py | Handshake complete |
| 4 | slam | ros2 run ros2_orb_slam3 mono_node_cpp | State: 2 |
| 5 | depth | ros2 run depth_anything_ros2 depth_node.py | Model loaded |
| 6 | fusion | ros2 run global_fusion global_fusion_node | GPS origin set |
| 7 | gt_bridge | ros2 run ros_gz_bridge parameter_bridge /ground_truth/... | Creating Bridge |
| 8 | qgc | ~/QGroundControl.AppImage | Connected to PX4 |

## Дополнительно (запускать вручную при необходимости)

### Сбор датасета для LSTM
```bash
source ~/drone-lstm-fusion/ros2_ws/install/setup.bash
ros2 run lstm_data_collector collector_node.py \
  --ros-args -p output_file:=/tmp/lstm_dataset.csv -p max_samples:=50000
```
Взлетай и летай активно — разные высоты, скорости, направления.

### Обучение LSTM
```bash
source ~/depth_anything_venv/bin/activate
cd ~/drone-lstm-fusion/training
python3 train_lstm.py --dataset /tmp/lstm_dataset.csv
```

## Проверочные команды

```bash
ros2 topic hz /imu               # ~85 Hz
ros2 topic hz /vio/odometry      # ~11 Hz
ros2 topic hz /depth/altitude    # ~6 Hz
ros2 topic hz /global_odometry   # ~78 Hz
ros2 topic hz /ground_truth/x500_mono_cam_0/odometry  # ~19 Hz
```

## Ключевые грабли

- GZ_SIM_SYSTEM_PLUGIN_PATH должен включать путь к ground_truth_plugin/lib
- Порядок source важен: сначала ROS2 setup.bash, потом venv (для depth ноды)
- Газебо должен быть запущен ДО PX4 (PX4_GZ_STANDALONE=1)
- IMU bridge слушает длинный путь топика с remapping на /imu
- Камера: pose=".15 0 .05 0 1.047 0" (+60° вниз, положительный знак!)
