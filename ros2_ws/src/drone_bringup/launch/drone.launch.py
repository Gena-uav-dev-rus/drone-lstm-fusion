from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Camera LEFT bridge
    camera_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='camera_bridge',
                arguments=[
                    '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                ],
                output='screen'
            )
        ]
    )

    # Camera RIGHT bridge (стерео пара)
    camera_right_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='camera_right_bridge',
                arguments=[
                    '/camera_right@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/camera_right_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                ],
                output='screen'
            )
        ]
    )

    # IMU bridge — поддерживаем оба имени модели (mono_cam и stereo_cam)
    imu_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='imu_bridge',
                arguments=[
                    '/world/baylands/model/x500_stereo_cam_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                ],
                remappings=[
                    ('/world/baylands/model/x500_stereo_cam_0/link/base_link/sensor/imu_sensor/imu', '/imu'),
                ],
                output='screen'
            )
        ]
    )

    # Gazebo → ros2_orb_slam3 драйвер
    mono_driver = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='drone_bringup',
                executable='gazebo_mono_driver',
                name='gazebo_mono_driver',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        camera_bridge,
        camera_right_bridge,
        imu_bridge,
        mono_driver,
    ])
