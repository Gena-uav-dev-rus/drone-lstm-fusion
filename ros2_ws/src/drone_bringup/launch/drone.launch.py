from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    # XRCE-DDS Agent
    xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # Camera bridge
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

    # IMU bridge
    imu_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='imu_bridge',
                arguments=[
                    '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
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
        xrce_agent,
        camera_bridge,
        imu_bridge,
        mono_driver,
    ])
