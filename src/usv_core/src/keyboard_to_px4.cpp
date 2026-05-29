#include <rclcpp/rclcpp.hpp>
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
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std::chrono_literals;

enum class BridgeState {
    INIT_HEARTBEAT,
    ARM_AND_SWITCH,
    ACTIVE
};

class UsvOffboardTest : public rclcpp::Node
{
public:
    UsvOffboardTest() : Node("usv_offboard_test"), current_yaw_ned_(0.0), target_vx_(0.0), target_wz_(0.0)
    {
        px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            std::bind(&UsvOffboardTest::odomCallback, this, std::placeholders::_1));

        px4_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", rclcpp::QoS(10).best_effort(),
            std::bind(&UsvOffboardTest::statusCallback, this, std::placeholders::_1));

        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        bridge_state_ = BridgeState::INIT_HEARTBEAT;
        init_counter_ = 0;
        
        auto now_ns = this->get_clock()->now().nanoseconds();
        last_cmd_send_time_ns_.store(now_ns);
        last_turn_key_time_ = this->get_clock()->now();

        control_timer_ = this->create_wall_timer(50ms, std::bind(&UsvOffboardTest::controlLoop, this));

        // 启动键盘监听线程
        keyboard_thread_ = std::thread(&UsvOffboardTest::keyboardLoop, this);

        RCLCPP_INFO(this->get_logger(), "\033[1;32m[INIT] USV Keyboard Test Node Started \033[0m");
        RCLCPP_INFO(this->get_logger(), "\033[1;36mControls:\n [W]: 加速 (+0.2, Max 2.0)\n [S]: 减速 (-0.2, Min 0.0)\n [A/D]: 左右转 (长按, 0.785 rad/s)\n [Space]: 紧急停船 (归零)\033[0m");
    }

    ~UsvOffboardTest() {
        // 节点销毁时恢复终端设置
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }

private:
    void keyboardLoop() {
        // 备份并修改终端设置以实现无阻塞按键读取
        tcgetattr(STDIN_FILENO, &original_termios_);
        struct termios newt = original_termios_;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

        while (rclcpp::ok()) {
            char c;
            int n = read(STDIN_FILENO, &c, 1);
            if (n > 0) {
                auto now = this->get_clock()->now();
                if (c == 'w' || c == 'W') {
                    double v = target_vx_.load();
                    v += 0.2;
                    if (v > 0.0 && v < 0.2) v = 0.2; // 最低速度0.2
                    if (v > 2.0) v = 2.0;            // 上限2.0
                    target_vx_.store(v);
                } 
                else if (c == 's' || c == 'S') {
                    double v = target_vx_.load();
                    v -= 0.2;
                    if (v < 0.0) v = 0.0;            // 最小值为0.0，不能倒车
                    target_vx_.store(v);
                } 
                else if (c == ' ') {
                    target_vx_.store(0.0);
                    target_wz_.store(0.0);
                } 
                else if (c == 'a' || c == 'A') {
                    target_wz_.store(0.785);         // 左转 (ROS标准中Z轴正向为左)
                    last_turn_key_time_ = now;
                } 
                else if (c == 'd' || c == 'D') {
                    target_wz_.store(-0.785);        // 右转
                    last_turn_key_time_ = now;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
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
                // 判断按键松开：如果距离最后一次接收到 A/D 信号超过 150ms，则视作按键已松开
                if ((now - last_turn_key_time_).seconds() > 0.15) {
                    target_wz_.store(0.0);
                }

                double raw_vx = target_vx_.load();
                double raw_wz = target_wz_.load();

                // === 双发 USV 测试特性 ===
                // 如果当前正在给转弯指令，但速度为0（静止状态），强制给 0.2 的向前油门以便电机能够转动船体
                if (std::abs(raw_wz) > 0.01 && raw_vx < 0.2) {
                    raw_vx = 0.2;
                }

                // 一阶低通滤波 (Smoothing)
                const double alpha = 0.3; 
                smooth_vx_ = alpha * raw_vx + (1.0 - alpha) * smooth_vx_;
                smooth_wz_ = alpha * raw_wz + (1.0 - alpha) * smooth_wz_;

                // 最小死区过滤，避免浮点数残留
                double vx = (std::abs(smooth_vx_) > 0.01) ? smooth_vx_ : 0.0;
                double wz = (std::abs(smooth_wz_) > 0.01) ? smooth_wz_ : 0.0;

                // 转向与抗漂移逻辑
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

        // 控制台状态打印 (1Hz 节流)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "[KEYBOARD STATE]  target_vx: %.2f, target_wz: %.2f\n"
            "  [PX4_CMD OUT] vel(N: %.2f, E: %.2f, D: %.2f), yaw: %.2f, yawspeed: %.2f",
            target_vx_.load(), target_wz_.load(),
            setpoint_msg.velocity[0], setpoint_msg.velocity[1], setpoint_msg.velocity[2],
            setpoint_msg.yaw, setpoint_msg.yawspeed);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr px4_status_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    BridgeState bridge_state_;
    int init_counter_;
    
    double smooth_vx_{0.0};
    double smooth_wz_{0.0};
    
    std::atomic<int64_t> last_cmd_send_time_ns_;
    rclcpp::Time last_turn_key_time_;

    std::atomic<double> current_yaw_ned_;
    std::atomic<double> target_vx_;
    std::atomic<double> target_wz_;
    std::atomic<uint8_t> nav_state_{0};
    std::atomic<uint8_t> arming_state_{0};

    // 终端和键盘输入线程相关
    struct termios original_termios_;
    std::thread keyboard_thread_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UsvOffboardTest>());
    rclcpp::shutdown();
    return 0;
}