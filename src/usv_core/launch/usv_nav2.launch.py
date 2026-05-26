import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 路径定义
    usv_core_dir = get_package_share_directory('usv_core')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    params_file = os.path.join(usv_core_dir, 'config', 'usv_nav2_params.yaml')
    
    # 2. Launch 配置变量
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 3. 核心节点启动命令 (调用官方 navigation_launch.py, 完美避开 map_server 和 amcl)
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'       # 自动激活所有 Lifecycle 节点
        }.items()
    )

    # 4. 构建 LaunchDescription
    ld = LaunchDescription()
    
    # 声明启动参数，支持终端动态覆盖 (例如: ros2 launch usv_core usv_nav2.launch.py use_sim_time:=true)
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))
        
    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'))

    # 将 Nav2 进程加入启动队列
    ld.add_action(nav2_cmd)

    return ld