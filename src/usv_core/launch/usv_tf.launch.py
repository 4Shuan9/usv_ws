from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            # 明确指定参数，保证雷达在 base_link 之上 20cm
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser_link'],
            output='screen'
        )
    ])