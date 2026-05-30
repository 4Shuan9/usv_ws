#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <csignal>
#include <thread>
#include <chrono>
#include <algorithm> // for std::clamp

using namespace std::chrono_literals;

class CmdVelToPx4 : public rclcpp::Node
{
public:
    CmdVelToPx4() : Node("cmd_vel_to_px4"), 
                    current_vx_(0.0), current_vy_(0.0), 
                    target_vx_(0.0), target_vy_(0.0),
                    filtered_raw_vx_(0.0), filtered_raw_vy_(0.0)
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CmdVelToPx4::cmdVelCallback, this, std::placeholders::_1));
            
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos, std::bind(&CmdVelToPx4::vehicleStatusCallback, this, std::placeholders::_1));

        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        control_timer_ = this->create_wall_timer(100ms, std::bind(&CmdVelToPx4::controlLoop, this));
        state_timer_ = this->create_wall_timer(1000ms, std::bind(&CmdVelToPx4::stateGuardLoop, this));

        RCLCPP_INFO(this->get_logger(), "\033[1;32m[INIT] Nav2_To_PX4 Offboard Bridge Started. Waiting for commands...\033[0m");
    }

    void emergencyShutdown()
    {
        RCLCPP_WARN(this->get_logger(), "\033[1;31mCtrl+C Detected! Switching to MANUAL and DISARMING...\033[0m");
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
        std::this_thread::sleep_for(50ms);
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        std::this_thread::sleep_for(50ms);
    }

private:
    uint8_t nav_state_{0};
    uint8_t arming_state_{0};
    bool has_entered_offboard_{false}; // 单次进入 Offboard 标志位
    
    // 速度与滤波变量
    double current_vx_, current_vy_;
    double target_vx_, target_vy_;
    double filtered_raw_vx_, filtered_raw_vy_;

    // ================= 核心调参宏 =================
    const double ALPHA = 0.5;             // /cmd_vel 的一阶低通滤波系数 (越小越平滑，但延迟越大)
    const double ACCEL_LIMIT = 0.5;       // 每秒最大加速度 0.5 m/s^2
    const double DEADZONE_X = 0.4;        // X轴死区
    const double DEADZONE_Y = 0.2;        // Y轴死区
    const double LEFT_FEEDFORWARD = -0.1; // 向左前馈补偿（PX4 FRD中Vy负值为左）
    
    // 新增限幅与缩放参数
    const double SCALE_X = 1.5;           // X轴(线速度)放大倍数
    const double MAX_VEL_X = 2.0;         // X轴(线速度)最大绝对值限幅 (m/s)
    const double MAX_VEL_Y = 1.0;         // Y轴(角速度转化的横向速度)最大绝对值限幅 (m/s)
    // ==============================================

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;

    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        nav_state_ = msg->nav_state;
        arming_state_ = msg->arming_state;

        // 只要成功进入过一次 Offboard，就上锁，防止后续遥控器切出后节点死磕抢夺
        if (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
            has_entered_offboard_ = true;
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 1. 滤波：平滑 Nav2 的剧烈跳动指令
        filtered_raw_vx_ = ALPHA * msg->linear.x + (1.0 - ALPHA) * filtered_raw_vx_;
        filtered_raw_vy_ = ALPHA * (-msg->angular.z) + (1.0 - ALPHA) * filtered_raw_vy_;

        // 2. 缩放与限幅：使用核心调参宏进行控制
        double raw_vx = filtered_raw_vx_ * SCALE_X;
        raw_vx = std::clamp(raw_vx, -MAX_VEL_X, MAX_VEL_X);
        
        double raw_vy = std::clamp(filtered_raw_vy_, -MAX_VEL_Y, MAX_VEL_Y);

        // 3. 差速模型联动补全 (只给角速度不给线速度船不动的情况)
        if (std::abs(raw_vy) > 0.01 && std::abs(raw_vx) < 0.01) {
            raw_vx = 0.15; 
        }

        // 4. 应用独立死区
        target_vx_ = applyDeadzone(raw_vx, DEADZONE_X);
        target_vy_ = applyDeadzone(raw_vy, DEADZONE_Y);

        // 5. 硬件物理纠偏 (写在死区下方)
        // 只要有实际的速度指令输出（船需要动），就叠加向左的固定前馈
        if (std::abs(target_vx_) > 0.001 || std::abs(target_vy_) > 0.001) {
            target_vy_ += LEFT_FEEDFORWARD;
        }
    }

    double applyDeadzone(double val, double deadzone_thresh)
    {
        if (std::abs(val) < 0.01) {
            return 0.0;
        }
        if (std::abs(val) < deadzone_thresh) {
            return (val > 0) ? deadzone_thresh : -deadzone_thresh;
        }
        return val;
    }

    void controlLoop()
    {
        // 维持 Offboard 心跳
        px4_msgs::msg::OffboardControlMode mode_msg;
        mode_msg.position = false;
        mode_msg.velocity = true;
        mode_msg.acceleration = false;
        mode_msg.attitude = false;
        mode_msg.body_rate = false;
        mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(mode_msg);

        // 运动平滑处理
        double dt = 0.1; // 10Hz 控制环
        double max_dv = ACCEL_LIMIT * dt;

        current_vx_ = smoothVelocity(current_vx_, target_vx_, max_dv);
        current_vy_ = smoothVelocity(current_vy_, target_vy_, max_dv);

        // 发布轨迹设定点
        px4_msgs::msg::TrajectorySetpoint setpoint;
        setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        setpoint.velocity[0] = current_vx_;
        setpoint.velocity[1] = current_vy_;
        setpoint.velocity[2] = 0.0;
        
        setpoint.position = {NAN, NAN, NAN};
        setpoint.acceleration = {NAN, NAN, NAN};
        setpoint.yaw = NAN;
        setpoint.yawspeed = NAN; 
        trajectory_setpoint_pub_->publish(setpoint);
    }

    double smoothVelocity(double current, double target, double max_step)
    {
        if (current < target) {
            return std::min(current + max_step, target);
        } else if (current > target) {
            return std::max(current - max_step, target);
        }
        return current;
    }

    void stateGuardLoop()
    {
        // 如果从未进入过 Offboard，且当前不在 Offboard 状态，则发起切换
        if (!has_entered_offboard_ && nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
            RCLCPP_WARN(this->get_logger(), "Attempting initial switch to Offboard mode...");
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        }
        
        // Arming 逻辑同样受到 !has_entered_offboard_ 的保护，防止遥控器上锁后代码强制重新解锁
        if (!has_entered_offboard_ && arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
            RCLCPP_WARN(this->get_logger(), "Vehicle disarmed. Attempting initial Arming...");
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        }
    }

    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        px4_msgs::msg::VehicleCommand msg;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }
};

std::shared_ptr<CmdVelToPx4> node_ptr = nullptr;

void signalHandler(int signum)
{
    (void)signum;
    if (node_ptr) {
        node_ptr->emergencyShutdown();
    }
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    node_ptr = std::make_shared<CmdVelToPx4>();
    
    std::signal(SIGINT, signalHandler);

    rclcpp::spin(node_ptr);
    return 0;
}