import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    usv_core_dir = get_package_share_directory('usv_core')

    # ================= 模式切换参数 =================
    # 声明室内模式参数，默认值为 false (即默认处于室外模式，依赖 GPS)
    indoor_mode_arg = DeclareLaunchArgument(
        'indoor_mode',
        default_value='false',
        description='Set to true for indoor environment (enables rf2o visual odometry)'
    )
    indoor_mode = LaunchConfiguration('indoor_mode')
    # ============================================

    # 1. MicroXRCEAgent (带启动前清理)
    micrortps_agent = ExecuteProcess(
        cmd=['sh', '-c', 'pkill -x MicroXRCEAgent || true; MicroXRCEAgent serial --dev /dev/ttyS7 -b 1500000'],
        output='screen',
        name='micro_xrce_agent'
    )

    # 2. 国科光芯雷达节点
    lidar_node = ExecuteProcess(
        cmd=['ros2', 'run', 'm1ct_d2', 'm1ct_d2'],
        output='screen',
        name='m1ct_d2_node'
    )

    # 3. 雷达看门狗节点 (监控并重启假死的雷达)
    lidar_watchdog_node = Node(
        package='usv_core',
        executable='lidar_watchdog.py',
        name='lidar_watchdog',
        output='screen'
    )

    # 4. 静态 TF (发布 base_link -> laser_link)
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(usv_core_dir, 'launch', 'usv_tf.launch.py')
        )
    )

    # ================= 里程计隔离逻辑 =================
    # 5. 启动 rf2o 激光雷达里程计 (仅在 indoor_mode:=true 时启动)
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        condition=IfCondition(indoor_mode),  # <-- 条件启动
        ros_arguments=['--log-level', 'WARN'],
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0
        }]
    )

    # 6. 启动 rf2o 到 PX4 的桥接节点 (仅在 indoor_mode:=true 时启动)
    rf2o_to_px4_node = Node(
        package='usv_core',
        executable='rf2o_to_px4',
        name='rf2o_to_px4',
        output='screen',
        condition=IfCondition(indoor_mode),  # <-- 条件启动
        ros_arguments=['--log-level', 'WARN']
    )
    # ============================================

    # 7. PX4 到 Nav2 的桥接节点 (全局必须，为 Nav2 提供 Odom)
    px4_to_nav2_node = Node(
        package='usv_core',
        executable='px4_to_nav2',
        name='px4_to_nav2',
        output='screen'
    )

    # 构建并返回 Launch 描述
    ld = LaunchDescription()
    ld.add_action(indoor_mode_arg)
    ld.add_action(micrortps_agent)
    ld.add_action(lidar_node)
    ld.add_action(lidar_watchdog_node)  # 加入看门狗
    ld.add_action(tf_launch)
    ld.add_action(rf2o_node)            # 条件执行
    ld.add_action(rf2o_to_px4_node)     # 条件执行
    ld.add_action(px4_to_nav2_node)

    return ld