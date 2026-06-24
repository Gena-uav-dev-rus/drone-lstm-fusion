#!/bin/bash
# Запись rosbag во время полёта для анализа крашей
# Записывает VIO, attitude, local_position, visual_odometry и PX4 статус

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_PATH=~/drone-lstm-fusion/bags/flight_${TIMESTAMP}

mkdir -p ~/drone-lstm-fusion/bags

echo "Recording to: ${BAG_PATH}"
echo "Press Ctrl+C to stop recording"

ros2 bag record \
    /vio/odometry \
    /depth/altitude \
    /global_odometry \
    /fmu/out/vehicle_attitude \
    /fmu/out/vehicle_local_position \
    /fmu/out/vehicle_gps_position \
    /fmu/out/vehicle_status \
    /fmu/out/timesync_status \
    /fmu/in/vehicle_visual_odometry \
    /lstm_noise/gps_variance \
    /lstm_noise/vio_variance \
    /lstm_noise/depth_variance \
    -o ${BAG_PATH}
