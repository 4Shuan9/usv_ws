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
        // --- 1. 参数声明与初始化 ---
        declareParameters();

        // --- 2. QOS 配置 ---
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

        // --- 3. 订阅者与发布者初始化 ---
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CmdVelToPx4::cmdVelCallback, this, std::placeholders::_1));
            
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos, std::bind(&CmdVelToPx4::vehicleStatusCallback, this, std::placeholders::_1));

        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        // --- 4. 定时器初始化 ---
        control_timer_ = this->create_wall_timer(100ms, std::bind(&CmdVelToPx4::controlLoop, this));
        state_timer_ = this->create_wall_timer(1000ms, std::bind(&CmdVelToPx4::stateGuardLoop, this));

        // --- 5. 动态调参回调绑定 ---
        param_sub_ = this->add_on_set_parameters_callback(
            std::bind(&CmdVelToPx4::parametersCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "\033[1;32m[INIT] Nav2_To_PX4 Offboard Bridge Started. Waiting for commands...\033[0m");
    }

    // --- 紧急制动接口 ---
    void emergencyShutdown()
    {
        RCLCPP_WARN(this->get_logger(), "\033[1;31mCtrl+C Detected! Switching to MANUAL and DISARMING...\033[0m");
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
        std::this_thread::sleep_for(50ms);
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        std::this_thread::sleep_for(50ms);
    }

private:
    // --- 状态标志位 ---
    uint8_t nav_state_{0};
    uint8_t arming_state_{0};
    bool has_entered_offboard_{false};
    
    // --- 速度与滤波缓存变量 ---
    double current_vx_, current_vy_;
    double target_vx_, target_vy_;
    double filtered_raw_vx_, filtered_raw_vy_;

    // --- 动态调参变量 (替代原核心宏) ---
    double alpha_;             // 一阶低通滤波系数
    double coeff_x_;           // X轴系数 (线速度)
    double coeff_y_;           // Y轴系数 (角速度缩放)
    double max_vel_x_;         // X轴最大绝对值限幅
    double max_vel_y_;         // Y轴最大绝对值限幅
    double min_vel_x_;         // X轴最低启动速度
    double min_vel_y_;         // Y轴最低启动速度
    double accel_limit_x_;     // X轴线加速度限制
    double accel_limit_y_;     // Y轴角加速度限制
    double left_feedforward_;  // 向左前馈补偿

    // --- ROS 2 节点句柄 ---
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_sub_;

    // --- 内部方法：声明并初始化参数 ---
    void declareParameters()
    {
        this->declare_parameter("alpha", 0.8);
        this->declare_parameter("coeff_x", 2.5);
        this->declare_parameter("coeff_y", 0.4);
        this->declare_parameter("max_vel_x", 2.0);
        this->declare_parameter("max_vel_y", 0.8);
        this->declare_parameter("min_vel_x", 0.3);
        this->declare_parameter("min_vel_y", 0.1);
        this->declare_parameter("accel_limit_x", 0.8);
        this->declare_parameter("accel_limit_y", 0.3);
        this->declare_parameter("left_feedforward", 0.0);

        alpha_ = this->get_parameter("alpha").as_double();
        coeff_x_ = this->get_parameter("coeff_x").as_double();
        coeff_y_ = this->get_parameter("coeff_y").as_double();
        max_vel_x_ = this->get_parameter("max_vel_x").as_double();
        max_vel_y_ = this->get_parameter("max_vel_y").as_double();
        min_vel_x_ = this->get_parameter("min_vel_x").as_double();
        min_vel_y_ = this->get_parameter("min_vel_y").as_double();
        accel_limit_x_ = this->get_parameter("accel_limit_x").as_double();
        accel_limit_y_ = this->get_parameter("accel_limit_y").as_double();
        left_feedforward_ = this->get_parameter("left_feedforward").as_double();
    }

    // --- 内部方法：动态参数回调 ---
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Parameters successfully updated";

        for (const auto &param : parameters) {
            if (param.get_name() == "alpha") alpha_ = param.as_double();
            else if (param.get_name() == "coeff_x") coeff_x_ = param.as_double();
            else if (param.get_name() == "coeff_y") coeff_y_ = param.as_double();
            else if (param.get_name() == "max_vel_x") max_vel_x_ = param.as_double();
            else if (param.get_name() == "max_vel_y") max_vel_y_ = param.as_double();
            else if (param.get_name() == "min_vel_x") min_vel_x_ = param.as_double();
            else if (param.get_name() == "min_vel_y") min_vel_y_ = param.as_double();
            else if (param.get_name() == "accel_limit_x") accel_limit_x_ = param.as_double();
            else if (param.get_name() == "accel_limit_y") accel_limit_y_ = param.as_double();
            else if (param.get_name() == "left_feedforward") left_feedforward_ = param.as_double();
            
            RCLCPP_INFO(this->get_logger(), "Parameter updated: %s = %f", param.get_name().c_str(), param.as_double());
        }
        return result;
    }

    // --- 回调：状态更新 ---
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        nav_state_ = msg->nav_state;
        arming_state_ = msg->arming_state;
        if (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
            has_entered_offboard_ = true;
        }
    }

    // --- 回调：速度指令处理 ---
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 1. 滤波处理
        filtered_raw_vx_ = alpha_ * msg->linear.x + (1.0 - alpha_) * filtered_raw_vx_;
        filtered_raw_vy_ = alpha_ * (-msg->angular.z) + (1.0 - alpha_) * filtered_raw_vy_;

        // 2. 系数缩放
        double raw_vx = filtered_raw_vx_ * coeff_x_;
        double raw_vy = filtered_raw_vy_ * coeff_y_;

        // 3. 绝对值限幅
        raw_vx = std::clamp(raw_vx, -max_vel_x_, max_vel_x_);
        raw_vy = std::clamp(raw_vy, -max_vel_y_, max_vel_y_);

        // 4. 原地旋转差速保护
        if (std::abs(raw_vy) > 0.01 && std::abs(raw_vx) < 0.01) {
            raw_vx = 0.15; 
        }

        // 5. 应用最低启动速度 (解决微调过猛问题)
        target_vx_ = applyMinStartingVelocity(raw_vx, min_vel_x_);
        target_vy_ = applyMinStartingVelocity(raw_vy, min_vel_y_);

        // 6. 硬件物理前馈纠偏
        if (std::abs(target_vx_) > 0.001 || std::abs(target_vy_) > 0.001) {
            target_vy_ += left_feedforward_;
        }
    }

    // --- 算法：最低启动速度计算 ---
    double applyMinStartingVelocity(double val, double min_vel)
    {
        if (std::abs(val) < 0.01) {
            return 0.0;
        }
        if (std::abs(val) < min_vel) {
            return (val > 0) ? min_vel : -min_vel;
        }
        return val;
    }

    // --- 算法：速度平滑 (加速度限制) ---
    double smoothVelocity(double current, double target, double max_step)
    {
        if (current < target) {
            return std::min(current + max_step, target);
        } else if (current > target) {
            return std::max(current - max_step, target);
        }
        return current;
    }

    // --- 任务：10Hz 控制主循环 ---
    void controlLoop()
    {
        // 维持心跳
        px4_msgs::msg::OffboardControlMode mode_msg;
        mode_msg.position = false;
        mode_msg.velocity = true;
        mode_msg.acceleration = false;
        mode_msg.attitude = false;
        mode_msg.body_rate = false;
        mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(mode_msg);

        // 加速度限制处理
        double dt = 0.1; // 10Hz
        double max_dv_x = accel_limit_x_ * dt;
        double max_dv_y = accel_limit_y_ * dt;

        current_vx_ = smoothVelocity(current_vx_, target_vx_, max_dv_x);
        current_vy_ = smoothVelocity(current_vy_, target_vy_, max_dv_y);

        // 发布设定点
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

    // --- 任务：1Hz 状态守护守护循环 ---
    void stateGuardLoop()
    {
        if (!has_entered_offboard_ && nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
            RCLCPP_WARN(this->get_logger(), "Attempting initial switch to Offboard mode...");
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        }
        
        if (!has_entered_offboard_ && arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
            RCLCPP_WARN(this->get_logger(), "Vehicle disarmed. Attempting initial Arming...");
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        }
    }

    // --- 工具：发布底层命令 ---
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