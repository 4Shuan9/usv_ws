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

    # 默认配置文件路径
    default_params_file = os.path.join(usv_core_dir, 'config', 'usv_nav2_params.yaml')
    default_bt_xml_file = os.path.join(usv_core_dir, 'config', 'usv_mapless_bt.xml')

    # 1. 创建 Launch 配置变量 (建立与外部传参的桥梁)
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')

    # 2. 声明启动参数 (设定默认值，不影响当前程序运行)
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to ROS2 parameters file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml',
        default_value=default_bt_xml_file,
        description='Full path to the behavior tree xml file to use')

    # 3. 启动 Nav2 导航栈
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml
        }.items()
    )

    # 4. 构建 Launch 描述并组装
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(nav2_cmd)

    return ld