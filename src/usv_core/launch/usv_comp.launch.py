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

    # 2. 雷达节点 m1ct_d2 (修改这里！使用 ExecuteProcess 模拟 ros2 run)
    lidar_node = ExecuteProcess(
        cmd=['ros2', 'run', 'm1ct_d2', 'm1ct_d2'],
        output='screen',
        name='m1ct_d2_node'
    )

    # 3. 静态 TF
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(usv_core_dir, 'launch', 'usv_tf.launch.py')
        )
    )

    # 4. PX4 to Nav2 桥接节点
    px4_odom_bridge_node = Node(
        package='usv_core',
        executable='px4_odom_bridge',
        name='px4_odom_bridge',
        output='screen'
    )

    # 构建并返回 Launch 描述
    ld = LaunchDescription()
    ld.add_action(micrortps_agent)
    ld.add_action(lidar_node)
    ld.add_action(tf_launch)
    ld.add_action(px4_odom_bridge_node)

    return ld