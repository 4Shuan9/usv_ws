import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    usv_core_dir = get_package_share_directory('usv_core')

    # 1. Nav2 导航栈 (引入已有的 launch 文件)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(usv_core_dir, 'launch', 'usv_nav2.launch.py')
        )
    )

    # 2. Offboard 控制节点 (下发控制指令到 PX4)
    cmd_vel_to_px4_node = Node(
        package='usv_core',
        executable='cmd_vel_to_px4',
        name='cmd_vel_to_px4',
        output='screen'
    )

    # 构建并返回 Launch 描述
    ld = LaunchDescription()
    ld.add_action(nav2_launch)
    ld.add_action(cmd_vel_to_px4_node)

    return ld