# USV 雷达与基座标系静态TF变换发布
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # base_link → laser_link 静态变换广播节点
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            # 参数格式：x y z roll pitch yaw 父坐标系 子坐标系
            # 雷达安装于base_link正上方0.2m，无角度偏移
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser_link'],
            output='screen'
        )
    ])