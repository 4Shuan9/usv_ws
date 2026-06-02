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
        keyboard_thread_ = std::thread(&UsvOffboardTest::keyboardLoop, this);

        RCLCPP_INFO(this->get_logger(), "\033[1;32m[INIT] USV Keyboard Test Node Started \033[0m");
        RCLCPP_INFO(this->get_logger(), "\033[1;36mControls:\n [W]: 加速 \n [S]: 减速 \n [Q/E]: 左右转 \n [Space]: 紧急停船\033[0m");
    }

    ~UsvOffboardTest() {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        if (keyboard_thread_.joinable()) keyboard_thread_.join();
    }

private:
    void keyboardLoop() {
        tcgetattr(STDIN_FILENO, &original_termios_);
        struct termios newt = original_termios_;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);

        while (rclcpp::ok()) {
            char c;
            int n = read(STDIN_FILENO, &c, 1);
            if (n > 0) {
                auto now = this->get_clock()->now();
                if (c == 'w' || c == 'W') {
                    double v = target_vx_.load();
                    v = std::min(v + 0.2, 2.0);
                    if (v > 0.0 && v < 0.2) v = 0.2;
                    target_vx_.store(v);
                } 
                else if (c == 's' || c == 'S') {
                    double v = target_vx_.load();
                    target_vx_.store(std::max(v - 0.2, 0.0));
                } 
                else if (c == ' ') {
                    target_vx_.store(0.0);
                    target_wz_.store(0.0);
                } 
                else if (c == 'q' || c == 'Q') {
                    target_wz_.store(0.785);
                    last_turn_key_time_ = now;
                } 
                else if (c == 'e' || c == 'E') {
                    target_wz_.store(-0.785);
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
        
        // 【坚决服从】：全局禁用 yaw
        setpoint_msg.yaw = NAN; 

        bool is_armed = (arming_state_.load() == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
        bool is_offboard = (nav_state_.load() == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);

        switch (bridge_state_) {
            case BridgeState::INIT_HEARTBEAT: {
                setpoint_msg.velocity = {0.0f, 0.0f, 0.0f};
                setpoint_msg.yawspeed = 0.0f;
                if (++init_counter_ >= 20) bridge_state_ = BridgeState::ARM_AND_SWITCH;
                break;
            }
            case BridgeState::ARM_AND_SWITCH: {
                setpoint_msg.velocity = {0.0f, 0.0f, 0.0f};
                setpoint_msg.yawspeed = 0.0f;
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
                if ((now - last_turn_key_time_).seconds() > 0.15) {
                    target_wz_.store(0.0);
                }

                double raw_vx = target_vx_.load();
                double raw_wz = target_wz_.load();

                if (std::abs(raw_wz) > 0.01 && raw_vx < 0.2) {
                    raw_vx = 0.2;
                }

                const double alpha = 0.3; 
                smooth_vx_ = alpha * raw_vx + (1.0 - alpha) * smooth_vx_;
                smooth_wz_ = alpha * raw_wz + (1.0 - alpha) * smooth_wz_;

                double vx = (std::abs(smooth_vx_) > 0.01) ? smooth_vx_ : 0.0;
                double wz = (std::abs(smooth_wz_) > 0.01) ? smooth_wz_ : 0.0;

                double current_yaw = current_yaw_ned_.load();
                
                // 【核心修复：防止 atan2(0,0) 回头跳变】
                // 如果用户没给速度 (vx = 0)，强制设为 1mm/s 的微小速度。
                // 这点速度无法克服水的阻力，船不会动；
                // 但它能保证 velocity 向量永远有效且指向当前船头，彻底废掉 PX4 底层烦人的自动归零(向北)对齐特性。
                double safe_vx = (std::abs(vx) > 0.01) ? vx : 0.001; 

                setpoint_msg.velocity[0] = safe_vx * cos(current_yaw);
                setpoint_msg.velocity[1] = safe_vx * sin(current_yaw);
                setpoint_msg.velocity[2] = 0.0;
                
                // 纯偏航角速度输入，完全由你的按键接管
                setpoint_msg.yawspeed = -wz; 
                break;
            }
        }
        
        trajectory_setpoint_pub_->publish(setpoint_msg);
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
    struct termios original_termios_;
    std::thread keyboard_thread_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UsvOffboardTest>());
    rclcpp::shutdown();
    return 0;
}