
### Окно 6 (НОВОЕ) — Depth Anything
```bash
source ~/drone-lstm-fusion/ros2_ws/install/setup.bash
source ~/depth_anything_venv/bin/activate
ros2 run depth_anything_ros2 depth_node.py
```
Ждём: `Model loaded successfully`, `DepthAnythingNode ready`

### Обновлённая нумерация окон в scripts/start_sim.sh
0: agent | 1: gazebo | 2: px4 | 3: bridge | 4: slam | 5: depth | 6: qgc
