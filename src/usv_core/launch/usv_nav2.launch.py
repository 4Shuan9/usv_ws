import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取 usv_core 的路径，找到参数文件
    usv_core_dir = get_package_share_directory('usv_core')
    nav2_params_path = os.path.join(usv_core_dir, 'config', 'usv_nav2_params.yaml')

    # 获取 nav2_bringup 包中的 navigation_launch.py
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_path,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        navigation_launch_cmd
    ])