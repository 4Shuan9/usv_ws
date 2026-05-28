#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>
#include <atomic>

using namespace std::chrono_literals;

enum class BridgeState {
    INIT_HEARTBEAT,
    ARM_AND_SWITCH,
    ACTIVE
};

class CmdVelToPx4 : public rclcpp::Node
{
public:
    CmdVelToPx4() : Node("cmd_vel_to_px4"), current_yaw_ned_(0.0), target_vx_flu_(0.0), target_wz_flu_(0.0)
    {
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CmdVelToPx4::cmdVelCallback, this, std::placeholders::_1));

        px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            std::bind(&CmdVelToPx4::odomCallback, this, std::placeholders::_1));

        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        bridge_state_ = BridgeState::INIT_HEARTBEAT;
        init_counter_ = 0;
        last_cmd_vel_time_ = this->get_clock()->now();

        control_timer_ = this->create_wall_timer(50ms, std::bind(&CmdVelToPx4::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "\033[1;32m[INIT] Velocity Bridge (Deadzone-Buster Edition) Started \033[0m");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_vx_flu_.store(std::clamp(msg->linear.x, -0.5, 1.5)); 
        target_wz_flu_.store(std::clamp(msg->angular.z, -1.5, 1.5));
        last_cmd_vel_time_ = this->get_clock()->now(); 
    }

    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        if (std::isnan(msg->q[0])) return;
        tf2::Quaternion q_px4(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
        double r, p, yaw_ned;
        tf2::Matrix3x3(q_px4).getRPY(r, p, yaw_ned);
        current_yaw_ned_.store(yaw_ned);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1; msg.param2 = param2; msg.command = command;
        msg.target_system = 1; msg.target_component = 1;
        msg.source_system = 1; msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }

    void controlLoop() {
        auto now = this->get_clock()->now();
        uint64_t timestamp = now.nanoseconds() / 1000; 

        px4_msgs::msg::OffboardControlMode mode_msg;
        mode_msg.timestamp = timestamp;
        mode_msg.position = false;
        mode_msg.velocity = true;
        mode_msg.acceleration = false;
        mode_msg.attitude = false;
        mode_msg.body_rate = false;
        offboard_control_mode_pub_->publish(mode_msg);

        px4_msgs::msg::TrajectorySetpoint setpoint_msg;
        setpoint_msg.timestamp = timestamp;
        setpoint_msg.position = {NAN, NAN, NAN};
        setpoint_msg.acceleration = {NAN, NAN, NAN};

        switch (bridge_state_) {
            case BridgeState::INIT_HEARTBEAT: {
                setpoint_msg.velocity = {0.0f, 0.0f, 0.0f};
                setpoint_msg.yaw = current_yaw_ned_.load();
                if (++init_counter_ >= 10) bridge_state_ = BridgeState::ARM_AND_SWITCH;
                break;
            }
            case BridgeState::ARM_AND_SWITCH: {
                setpoint_msg.velocity = {0.0f, 0.0f, 0.0f};
                setpoint_msg.yaw = current_yaw_ned_.load();
                RCLCPP_INFO(this->get_logger(), "\033[1;33m[BRIDGE] Switching to OFFBOARD & Arming...\033[0m");
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
                bridge_state_ = BridgeState::ACTIVE;
                break;
            }
            case BridgeState::ACTIVE: {
                double vx = target_vx_flu_.load();
                double wz = target_wz_flu_.load();
                
                // 看门狗：收不到 Nav2 指令强行归零
                if ((now - last_cmd_vel_time_).seconds() > 1.0)
                {
                    vx = 0.0; wz = 0.0;
                }

                double current_yaw = current_yaw_ned_.load();
                
                // --- 核心修复 1：目标航向积分器 ---
                static double target_yaw = current_yaw;
                if (std::abs(vx) > 0.01 || std::abs(wz) > 0.01) {
                    // Nav2给指令时，根据角速度推演目标朝向 (50ms积分)
                    target_yaw += (-wz) * 0.05; 
                    while (target_yaw > M_PI) target_yaw -= 2.0 * M_PI;
                    while (target_yaw < -M_PI) target_yaw += 2.0 * M_PI;
                } else {
                    target_yaw = current_yaw; // 停车时锁死当前航向
                }

                // --- 核心修复 2：击穿物理死区的"推背感"倍率 ---
                // 注意：如果船下水后冲得太猛，把这个 2.5 调小 (比如 1.5)
                double speed_boost = 2.5; 

                setpoint_msg.velocity[0] = vx * speed_boost * cos(target_yaw);
                setpoint_msg.velocity[1] = vx * speed_boost * sin(target_yaw);
                setpoint_msg.velocity[2] = 0.0;
                
                setpoint_msg.yaw = target_yaw;
                setpoint_msg.yawspeed = -wz; 
                break;
            }
        }
        trajectory_setpoint_pub_->publish(setpoint_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    BridgeState bridge_state_;
    int init_counter_;
    rclcpp::Time last_cmd_vel_time_;
    
    std::atomic<double> current_yaw_ned_;
    std::atomic<double> target_vx_flu_;
    std::atomic<double> target_wz_flu_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToPx4>());
    rclcpp::shutdown();
    return 0;
}