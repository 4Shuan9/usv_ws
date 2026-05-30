import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 包路径定义
    usv_core_dir = get_package_share_directory('usv_core')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 默认配置文件路径
    default_params_file = os.path.join(usv_core_dir, 'config', 'usv_nav2_params.yaml')
    default_bt_xml_file = os.path.join(usv_core_dir, 'config', 'usv_mapless_bt.xml')

    # 2. 创建 Launch 配置变量
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    # 🌟 核心修复：重新声明多点导航的配置变量
    default_nav_through_poses_bt_xml = LaunchConfiguration('default_nav_through_poses_bt_xml')

    # 3. 声明启动参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=default_params_file, description='Full path to ROS2 parameters file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup the nav2 stack')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml', default_value=default_bt_xml_file, description='Full path to the behavior tree xml file to use')

    # 🌟 核心修复：将多点导航默认文件也指向你的 usv_mapless_bt.xml
    declare_nav_through_poses_bt_xml_cmd = DeclareLaunchArgument(
        'default_nav_through_poses_bt_xml', default_value=default_bt_xml_file, description='Full path to the behavior tree xml file to use for navigate through poses')

    # 4. 启动 Nav2 导航栈
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
            # 🌟 核心修复：将参数向下传递给官方 launch，彻底阻断其加载默认包含 spin 的 XML
            'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml
        }.items()
    )

    # 5. 构建 Launch 描述并组装
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_nav_through_poses_bt_xml_cmd) 
    ld.add_action(nav2_cmd)

    return ld