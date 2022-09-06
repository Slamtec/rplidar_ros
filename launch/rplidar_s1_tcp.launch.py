from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'channel_type': 'tcp',
                'tcp_ip': '192.168.0.7',
                'tcp_port': 20108,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }]
        ),
    ])
