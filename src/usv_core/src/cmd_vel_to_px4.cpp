#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>
#include <atomic>

using namespace std::chrono_literals;

class CmdVelToPx4 : public rclcpp::Node
{
public:
    CmdVelToPx4() : Node("cmd_vel_to_px4"), current_yaw_ned_(0.0), target_vx_flu_(0.0), target_wz_flu_(0.0)
    {
        // === [1] 订阅器 (Subscribers) ===
        // Nav2 速度指令 (依赖默认的 Reliable QoS)
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelToPx4::cmdVelCallback, this, std::placeholders::_1));

        // PX4 里程计 (匹配 uXRCE-DDS 要求的 Best_Effort QoS)
        px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            std::bind(&CmdVelToPx4::odomCallback, this, std::placeholders::_1));

        // === [2] 发布器 (Publishers) ===
        // PX4 控制指令 (强制使用 Best_Effort 且队列深度设为 1，防止网络延迟导致的指令堆积)
        rclcpp::QoS px4_qos = rclcpp::QoS(1).best_effort();
        
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", px4_qos);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", px4_qos);

        // === [3] 定时器 (Timers) ===
        // 20Hz 控制主循环 (满足 PX4 Offboard 模式 >2Hz 的心跳要求)
        control_timer_ = this->create_wall_timer(
            50ms, std::bind(&CmdVelToPx4::controlLoop, this));

        last_cmd_vel_time_ = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "\033[1;32m[INIT] Velocity Bridge (cmd_vel -> PX4) Started \033[0m");
    }

private:
    // -----------------------------------------
    // 回调函数：缓存目标速度并执行安全限幅
    // -----------------------------------------
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_vx_flu_.store(std::clamp(msg->linear.x, -0.2, 1.0)); 
        target_wz_flu_.store(std::clamp(msg->angular.z, -1.0, 1.0));
        last_cmd_vel_time_ = this->get_clock()->now(); 
    }

    // -----------------------------------------
    // 回调函数：解析 PX4 四元数并提取全局 NED 偏航角
    // -----------------------------------------
    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        if (std::isnan(msg->q[0])) return;

        tf2::Quaternion q_px4(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
        double roll_frd, pitch_frd, yaw_ned;
        tf2::Matrix3x3(q_px4).getRPY(roll_frd, pitch_frd, yaw_ned);
        
        current_yaw_ned_.store(yaw_ned);
    }

    // -----------------------------------------
    // 核心主循环：处理心跳维持、看门狗干预与坐标系转换
    // -----------------------------------------
    void controlLoop()
    {
        auto now = this->get_clock()->now();
        uint64_t timestamp = now.nanoseconds() / 1000; 

        // 步骤 1: 发送 Offboard 模式心跳 (显式声明仅启用速度控制)
        px4_msgs::msg::OffboardControlMode mode_msg;
        mode_msg.timestamp = timestamp;
        mode_msg.position = false;
        mode_msg.velocity = true;
        mode_msg.acceleration = false;
        mode_msg.attitude = false;
        mode_msg.body_rate = false;
        offboard_control_mode_pub_->publish(mode_msg);

        // 步骤 2: 触发安全看门狗 (若 0.5s 内未接收到 cmd_vel，强制速度归零)
        double vx = target_vx_flu_.load();
        double wz = target_wz_flu_.load();
        
        if ((now - last_cmd_vel_time_).seconds() > 0.5) {
            vx = 0.0;
            wz = 0.0;
        }

        // 步骤 3: 构建 Setpoint 消息并执行坐标系转换
        px4_msgs::msg::TrajectorySetpoint setpoint_msg;
        setpoint_msg.timestamp = timestamp;

        // 屏蔽位置与加速度控制层级 (必须填充为 NAN)
        setpoint_msg.position = {NAN, NAN, NAN};
        setpoint_msg.acceleration = {NAN, NAN, NAN};
        setpoint_msg.yaw = NAN; 

        double yaw = current_yaw_ned_.load();

        // 平移速度变换: ROS2 机体 FLU 投影至 PX4 全局 NED (当前限 2D 移动)
        setpoint_msg.velocity[0] = vx * cos(yaw);
        setpoint_msg.velocity[1] = vx * sin(yaw);
        setpoint_msg.velocity[2] = 0.0;

        // 旋转速度变换: ROS2 FLU (逆时针为正) -> PX4 FRD (顺时针为正)
        setpoint_msg.yawspeed = -wz; 

        trajectory_setpoint_pub_->publish(setpoint_msg);
    }

    // === 类成员变量声明 ===
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Time last_cmd_vel_time_;
    std::atomic<double> current_yaw_ned_;
    std::atomic<double> target_vx_flu_;
    std::atomic<double> target_wz_flu_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToPx4>());
    rclcpp::shutdown();
    return 0;
}