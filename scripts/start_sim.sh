#!/bin/bash
export DISPLAY=:0
export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds
export GZ_SIM_SYSTEM_PLUGIN_PATH=~/drone-lstm-fusion/ros2_ws/install/ground_truth_plugin/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH

sudo pkill -9 -f px4 2>/dev/null
sudo pkill -9 -f gz 2>/dev/null
sudo pkill -9 -f MicroXRCE 2>/dev/null
sleep 3

tmux new-session -d -s drone -x 220 -y 50

# 0: agent
tmux rename-window -t drone:0 'agent'
tmux send-keys -t drone:0 'MicroXRCEAgent udp4 -p 8888' Enter
sleep 3

# 1: gazebo (headless)
tmux new-window -t drone -n 'gazebo'
tmux send-keys -t drone:1 'export DISPLAY=:0 && export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds && export GZ_SIM_SYSTEM_PLUGIN_PATH=~/drone-lstm-fusion/ros2_ws/install/ground_truth_plugin/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH && gz sim -r ~/PX4-Autopilot/Tools/simulation/gz/worlds/baylands.sdf' Enter
echo "Waiting for Gazebo 30 sec..."
sleep 30

# 2: px4
tmux new-window -t drone -n 'px4'
tmux send-keys -t drone:2 'export DISPLAY=:0 && export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds && cd ~/PX4-Autopilot && PX4_GZ_WORLD=baylands PX4_GZ_STANDALONE=1 make px4_sitl gz_x500_mono_cam' Enter
echo "Waiting for PX4 20 sec..."
sleep 20

# 3: bridge (camera + imu)
tmux new-window -t drone -n 'bridge'
tmux send-keys -t drone:3 'export DISPLAY=:0 && source ~/drone-lstm-fusion/ros2_ws/install/setup.bash && ros2 launch drone_bringup drone.launch.py' Enter
sleep 8

# 4: slam
tmux new-window -t drone -n 'slam'
tmux send-keys -t drone:4 'source ~/drone-lstm-fusion/ros2_ws/install/setup.bash && ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam -p voc_file_arg:=/home/sqwaer/drone-lstm-fusion/ros2_ws/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin -p settings_file_path_arg:=/home/sqwaer/drone-lstm-fusion/ros2_ws/src/ros2_orb_slam3/orb_slam3/config/Monocular/' Enter
sleep 5

# 5: depth
tmux new-window -t drone -n 'depth'
tmux send-keys -t drone:5 'source ~/drone-lstm-fusion/ros2_ws/install/setup.bash && source ~/depth_anything_venv/bin/activate && ros2 run depth_anything_ros2 depth_node.py' Enter
sleep 5

# 6: global_fusion
tmux new-window -t drone -n 'fusion'
tmux send-keys -t drone:6 'source ~/drone-lstm-fusion/ros2_ws/install/setup.bash && ros2 run global_fusion global_fusion_node' Enter
sleep 3

# 7: ground_truth bridge
tmux new-window -t drone -n 'gt_bridge'
tmux send-keys -t drone:7 'source ~/drone-lstm-fusion/ros2_ws/install/setup.bash && ros2 run ros_gz_bridge parameter_bridge /ground_truth/x500_mono_cam_0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry' Enter
sleep 2

# 8: lstm_gps
tmux new-window -t drone -n 'lstm_gps'
tmux send-keys -t drone:8 'source ~/drone-lstm-fusion/ros2_ws/install/setup.bash && source ~/depth_anything_venv/bin/activate && ros2 run lstm_noise_estimator gps_noise_node.py' Enter
sleep 3

# 9: lstm_vio
tmux new-window -t drone -n 'lstm_vio'
tmux send-keys -t drone:9 'source ~/drone-lstm-fusion/ros2_ws/install/setup.bash && source ~/depth_anything_venv/bin/activate && ros2 run lstm_noise_estimator vio_noise_node.py' Enter
sleep 3

# 10: lstm_depth
tmux new-window -t drone -n 'lstm_depth'
tmux send-keys -t drone:10 'source ~/drone-lstm-fusion/ros2_ws/install/setup.bash && source ~/depth_anything_venv/bin/activate && ros2 run lstm_noise_estimator depth_noise_node.py' Enter
sleep 3

# 11: qgc
tmux new-window -t drone -n 'qgc'
tmux send-keys -t drone:11 'export DISPLAY=:0 && ~/QGroundControl.AppImage 2>/dev/null' Enter

tmux attach -t drone
