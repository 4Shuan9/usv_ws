import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 包路径定义
    usv_core_dir = get_package_share_directory('usv_core')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 配置文件路径
    params_file = os.path.join(usv_core_dir, 'config', 'usv_nav2_params.yaml')
    bt_xml_file = os.path.join(usv_core_dir, 'config', 'usv_mapless_bt.xml')

    # Launch 配置变量
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 启动 Nav2 导航栈，强制使用自定义行为树
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'default_nav_to_pose_bt_xml': bt_xml_file
        }.items()
    )

    # 构建 Launch 描述
    ld = LaunchDescription()

    # 声明启动参数
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'))

    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to ROS2 parameters file'))

    ld.add_action(nav2_cmd)

    return ld