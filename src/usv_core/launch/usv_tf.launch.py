from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_broadcaster',
        arguments=[
            '--x', '0.0', 
            '--y', '0.0', 
            '--z', '0.2',
            '--roll', '0.0', 
            '--pitch', '0.0', 
            '--yaw', '0.0',
            '--frame-id', 'base_link',          # 父坐标系 (船体中心)
            '--child-frame-id', 'laser_link'    # 子坐标系 (雷达)
        ],
        output='screen'
    )

    return LaunchDescription([
        static_tf_node
    ])