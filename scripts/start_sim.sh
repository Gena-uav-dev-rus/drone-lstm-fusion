#!/bin/bash
export DISPLAY=:0
export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds

sudo pkill -9 -f px4 2>/dev/null
sudo pkill -9 -f gz 2>/dev/null
sudo pkill -9 -f MicroXRCE 2>/dev/null
sleep 3

tmux new-session -d -s drone -x 220 -y 50

tmux rename-window -t drone:0 'agent'
tmux send-keys -t drone:0 'MicroXRCEAgent udp4 -p 8888' Enter
sleep 3

tmux new-window -t drone -n 'gazebo'
tmux send-keys -t drone:1 'export DISPLAY=:0 && export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds && gz sim ~/PX4-Autopilot/Tools/simulation/gz/worlds/baylands.sdf -r' Enter
echo "Waiting for Gazebo 25 sec..."
sleep 25

tmux new-window -t drone -n 'px4'
tmux send-keys -t drone:2 'export DISPLAY=:0 && export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds && cd ~/PX4-Autopilot && PX4_GZ_WORLD=baylands PX4_GZ_STANDALONE=1 make px4_sitl gz_x500_mono_cam' Enter
echo "Waiting for PX4 20 sec..."
sleep 20

tmux new-window -t drone -n 'bridge'
tmux send-keys -t drone:3 'export DISPLAY=:0 && source ~/drone-lstm-fusion/ros2_ws/install/setup.bash && ros2 launch drone_bringup drone.launch.py' Enter
sleep 8

tmux new-window -t drone -n 'slam'
tmux send-keys -t drone:4 'source ~/drone-lstm-fusion/ros2_ws/install/setup.bash && ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam -p voc_file_arg:=/home/sqwaer/drone-lstm-fusion/ros2_ws/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin -p settings_file_path_arg:=/home/sqwaer/drone-lstm-fusion/ros2_ws/src/ros2_orb_slam3/orb_slam3/config/Monocular/' Enter
sleep 5

tmux new-window -t drone -n 'qgc'
tmux send-keys -t drone:5 'export DISPLAY=:0 && ~/QGroundControl.AppImage 2>/dev/null' Enter

tmux attach -t drone
