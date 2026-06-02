#!/usr/bin/env python3
import rclcpp
from rclcpp.node import Node
from sensor_msgs.msg import LaserScan
import subprocess
import time

class LidarWatchdog(Node):
    def __init__(self):
        super().__init__('lidar_watchdog')
        # 订阅激光雷达话题
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.last_msg_time = self.get_clock().now()
        
        # 定时器：每秒检查一次
        self.timer = self.create_timer(1.0, self.check_timeout)
        self.timeout_sec = 3.0  # 3秒无数据判定为假死
        self.restarting = False
        
        self.get_logger().info("\033[1;32m[INIT] Lidar Watchdog started. Monitoring /scan...\033[0m")

    def scan_callback(self, msg):
        # 只要收到数据就刷新时间戳
        self.last_msg_time = self.get_clock().now()
        self.restarting = False

    def check_timeout(self):
        if self.restarting:
            return
            
        now = self.get_clock().now()
        elapsed = (now - self.last_msg_time).nanoseconds / 1e9
        
        if elapsed > self.timeout_sec:
            self.get_logger().error(f"Lidar data timeout ({elapsed:.1f}s)! Restarting lidar node...")
            self.restarting = True
            self.restart_lidar()

    def restart_lidar(self):
        # 1. 杀掉现有的雷达进程 (匹配你在 launch 中运行的命令名)
        subprocess.run(['pkill', '-f', 'm1ct_d2'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        # 2. 等待1秒让串口/网口释放
        time.sleep(1.0) 
        
        # 3. 重新拉起雷达进程
        subprocess.Popen(['ros2', 'run', 'm1ct_d2', 'm1ct_d2'])
        self.get_logger().info("\033[1;33mLidar restart command issued.\033[0m")
        self.last_msg_time = self.get_clock().now()

def main(args=None):
    rclcpp.init(args=args)
    node = LidarWatchdog()
    try:
        rclcpp.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclcpp.shutdown()

if __name__ == '__main__':
    main()