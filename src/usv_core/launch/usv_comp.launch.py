import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    usv_core_dir = get_package_share_directory('usv_core')

    # 1. MicroXRCEAgent (带启动前清理)
    # 使用 sh -c 配合 pkill 先清理可能残留的后台进程，防止串口被占用，然后启动 Agent
    micrortps_agent = ExecuteProcess(
        cmd=['sh', '-c', 'pkill -f MicroXRCEAgent || true; MicroXRCEAgent serial --dev /dev/ttyS7 -b 1500000'],
        output='screen',
        name='micro_xrce_agent'
    )

    # 2. 雷达节点 m1ct_d2
    lidar_node = Node(
        package='m1ct_d2',
        executable='m1ct_d2',
        name='m1ct_d2_node',
        output='screen'
    )

    # 3. 静态 TF (引入已有的 launch 文件)
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(usv_core_dir, 'launch', 'usv_tf.launch.py')
        )
    )

    # 4. PX4 to Nav2 桥接节点 (里程计)
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