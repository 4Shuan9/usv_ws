#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
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

        px4_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", rclcpp::QoS(10).best_effort(),
            std::bind(&CmdVelToPx4::statusCallback, this, std::placeholders::_1));

        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        bridge_state_ = BridgeState::INIT_HEARTBEAT;
        init_counter_ = 0;
        
        auto now_ns = this->get_clock()->now().nanoseconds();
        last_cmd_vel_time_ns_.store(now_ns);
        last_cmd_send_time_ns_.store(now_ns);

        control_timer_ = this->create_wall_timer(50ms, std::bind(&CmdVelToPx4::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "\033[1;32m[INIT] USV Velocity Bridge Started (Unified Threshold & Auto-Throttle Edition) \033[0m");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 限制输入范围
        target_vx_flu_.store(std::clamp(msg->linear.x, -1.5, 1.5)); 
        target_wz_flu_.store(std::clamp(msg->angular.z, -1.5, 1.5));
        last_cmd_vel_time_ns_.store(this->get_clock()->now().nanoseconds()); 
    }

    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        if (std::isnan(msg->q[0])) return;
        tf2::Quaternion q_px4(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
        double r, p, yaw_ned;
        tf2::Matrix3x3(q_px4).getRPY(r, p, yaw_ned);
        current_yaw_ned_.store(yaw_ned);
    }

    void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        nav_state_.store(msg->nav_state);
        arming_state_.store(msg->arming_state);
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

        // Offboard 控制模式心跳
        px4_msgs::msg::OffboardControlMode mode_msg;
        mode_msg.timestamp = timestamp;
        mode_msg.velocity = true;
        offboard_control_mode_pub_->publish(mode_msg);

        px4_msgs::msg::TrajectorySetpoint setpoint_msg;
        setpoint_msg.timestamp = timestamp;
        setpoint_msg.position = {NAN, NAN, NAN};
        setpoint_msg.acceleration = {NAN, NAN, NAN};

        bool is_armed = (arming_state_.load() == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
        bool is_offboard = (nav_state_.load() == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);

        switch (bridge_state_) {
            case BridgeState::INIT_HEARTBEAT: {
                setpoint_msg.velocity = {0.0f, 0.0f, 0.0f};
                setpoint_msg.yaw = current_yaw_ned_.load();
                if (++init_counter_ >= 20) bridge_state_ = BridgeState::ARM_AND_SWITCH;
                break;
            }
            case BridgeState::ARM_AND_SWITCH: {
                setpoint_msg.velocity = {0.0f, 0.0f, 0.0f};
                setpoint_msg.yaw = current_yaw_ned_.load();
                if (is_armed && is_offboard) bridge_state_ = BridgeState::ACTIVE;
                else {
                    int64_t last_send_ns = last_cmd_send_time_ns_.load();
                    if ((now.nanoseconds() - last_send_ns) > 1e9) {
                        if (!is_offboard) publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                        if (is_offboard && !is_armed) publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
                        last_cmd_send_time_ns_.store(now.nanoseconds());
                    }
                }
                break;
            }
            case BridgeState::ACTIVE: {
                // --- 1. 一阶低通滤波 (Smoothing) ---
                const double alpha = 0.3; 
                double raw_vx = target_vx_flu_.load();
                double raw_wz = target_wz_flu_.load();

                // 看门狗逻辑：超时停止
                if ((now.nanoseconds() - last_cmd_vel_time_ns_.load()) > 1e9) {
                    raw_vx = 0.0; raw_wz = 0.0;
                }

                // 滤波更新
                smooth_vx_ = alpha * raw_vx + (1.0 - alpha) * smooth_vx_;
                smooth_wz_ = alpha * raw_wz + (1.0 - alpha) * smooth_wz_;

                // --- 2. 统一阈值介入 (Min Threshold 设为 0.3) ---
                auto apply_min_threshold = [](double val, double threshold) -> double {
                    if (std::abs(val) > 0.01) { // 摇杆死区过滤，防止回中时出现噪音
                        if (std::abs(val) < threshold) return (val > 0) ? threshold : -threshold;
                        return val;
                    } else {
                        return 0.0;
                    }
                };

                double vx = apply_min_threshold(smooth_vx_, 0.3);      
                double wz = apply_min_threshold(smooth_wz_, 0.3);      

                // --- 3. 修复：单转弯指令自动补油门 ---
                // 如果只给了旋转指令 (wz有效)，但没有给前进油门 (原始摇杆滤波值接近0)
                // 强制注入 0.3 的前进速度，以克服 USV 原地转向的水阻
                if (std::abs(wz) > 0.01 && std::abs(smooth_vx_) <= 0.01) {
                    vx = 0.3; 
                }

                // --- 4. 转向与抗漂移逻辑 ---
                double current_yaw = current_yaw_ned_.load();
                static double target_yaw = current_yaw;
                
                if (std::abs(vx) > 0.01 || std::abs(wz) > 0.01) {
                    target_yaw += (-wz) * 0.05; 
                    double yaw_error = current_yaw - target_yaw;
                    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
                    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
                    target_yaw += yaw_error * 0.05;
                } else {
                    target_yaw = current_yaw; 
                }

                setpoint_msg.velocity[0] = vx * cos(target_yaw);
                setpoint_msg.velocity[1] = vx * sin(target_yaw);
                setpoint_msg.velocity[2] = 0.0;
                setpoint_msg.yaw = target_yaw;
                setpoint_msg.yawspeed = -wz; 
                break;
            }
        }
        
        trajectory_setpoint_pub_->publish(setpoint_msg);

        // --- 1Hz 频率同步打印状态 ---
        if ((now.nanoseconds() - last_cmd_vel_time_ns_.load()) <= 1e9) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "[CMD_VEL IN]  vx: %.2f, wz: %.2f\n"
                "  [PX4_CMD OUT] vel(N: %.2f, E: %.2f, D: %.2f), yaw: %.2f, yawspeed: %.2f",
                target_vx_flu_.load(), target_wz_flu_.load(),
                setpoint_msg.velocity[0], setpoint_msg.velocity[1], setpoint_msg.velocity[2],
                setpoint_msg.yaw, setpoint_msg.yawspeed);
        }
    }

    // ROS 成员
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr px4_status_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    BridgeState bridge_state_;
    int init_counter_;
    
    // 平滑处理历史变量
    double smooth_vx_{0.0};
    double smooth_wz_{0.0};
    
    std::atomic<int64_t> last_cmd_vel_time_ns_;
    std::atomic<int64_t> last_cmd_send_time_ns_;
    std::atomic<double> current_yaw_ned_;
    std::atomic<double> target_vx_flu_;
    std::atomic<double> target_wz_flu_;
    std::atomic<uint8_t> nav_state_{0};
    std::atomic<uint8_t> arming_state_{0};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToPx4>());
    rclcpp::shutdown();
    return 0;
}