import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    usv_core_dir = get_package_share_directory('usv_core')

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

    # 3. 静态 TF (发布 base_link -> laser_link)
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(usv_core_dir, 'launch', 'usv_tf.launch.py')
        )
    )

    # ================= 新增部分 =================
    
    # 4. 启动 rf2o 激光雷达里程计
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        ros_arguments=['--log-level', 'WARN'],
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,         # 绝对不能发TF，交给飞控桥接器发
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0                 # 你的雷达是 10Hz，这里匹配为 10.0
        }]
    )

    # 5. 启动 rf2o 到 PX4 的桥接节点 (注入视觉里程计)
    rf2o_to_px4_node = Node(
        package='usv_core',
        executable='rf2o_to_px4',
        name='rf2o_to_px4',
        output='screen',
        ros_arguments=['--log-level', 'WARN']
    )

    # ============================================

    # 6. PX4 到 Nav2 的桥接节点
    px4_to_nav2_node = Node(
        package='usv_core',
        executable='px4_to_nav2',
        name='px4_to_nav2',
        output='screen'
    )

    # 构建并返回 Launch 描述
    ld = LaunchDescription()
    ld.add_action(micrortps_agent)
    ld.add_action(lidar_node)
    ld.add_action(tf_launch)
    ld.add_action(rf2o_node)          # 启动 2D 激光里程计
    ld.add_action(rf2o_to_px4_node)   # 激光里程计 -> 飞控
    ld.add_action(px4_to_nav2_node)   # 飞控 (融合后) -> ROS2 导航栈

    return ld