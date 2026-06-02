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
    UsvOffboardTest() : Node("usv_offboard_test"), 
        current_yaw_ned_(0.0), target_vx_flu_(0.0), target_vy_flu_(0.0), target_wz_flu_(0.0)
    {
        px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            std::bind(&UsvOffboardTest::odomCallback, this, std::placeholders::_1));

        px4_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", rclcpp::QoS(10).best_effort(),
            std::bind(&UsvOffboardTest::statusCallback, this, std::placeholders::_1));

        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        bridge_state_ = BridgeState::INIT_HEARTBEAT;
        init_counter_ = 0;
        last_cmd_send_time_ns_.store(this->get_clock()->now().nanoseconds());

        control_timer_ = this->create_wall_timer(50ms, std::bind(&UsvOffboardTest::controlLoop, this));

        printWelcomeMenu();

        keep_running_ = true;
        keyboard_thread_ = std::thread(&UsvOffboardTest::keyboardLoop, this);
    }

    ~UsvOffboardTest() {
        keep_running_ = false;
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        printf("\n\033[1;31m[INFO] USV 控制节点已退出。\033[0m\n");
    }

private:
    void printWelcomeMenu() {
        RCLCPP_INFO(this->get_logger(), "\033[1;32m 🚀 USV 纯手动遥控节点就绪 (纯角速度控制) 🚀 \033[0m");
        RCLCPP_INFO(this->get_logger(), " [\033[1;33m W / S \033[0m] : 前后开 (Vx)");
        RCLCPP_INFO(this->get_logger(), " [\033[1;33m A / D \033[0m] : 左右平移 (Vy) - 你的设置，原封不动");
        RCLCPP_INFO(this->get_logger(), " [\033[1;33m Q / E \033[0m] : 左转 / 右转 (纯角速度控制)");
        RCLCPP_INFO(this->get_logger(), " [\033[1;31m 空 格 \033[0m] : 紧急刹车");
        printf("\n等待按键输入...\n");
    }

    void keyboardLoop() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);

        const double STEP = 0.05;
        const double LIMIT = 5.0;

        while (keep_running_ && rclcpp::ok()) {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0) {
                double vx = target_vx_flu_.load();
                double vy = target_vy_flu_.load();
                double wz = target_wz_flu_.load();

                if      (c == 'w' || c == 'W') vx = std::min(vx + STEP, LIMIT);
                else if (c == 's' || c == 'S') vx = std::max(vx - STEP, -LIMIT);
                else if (c == 'a' || c == 'A') vy = std::min(vy + STEP, LIMIT);  
                else if (c == 'd' || c == 'D') vy = std::max(vy - STEP, -LIMIT); 
                else if (c == 'q' || c == 'Q') wz = std::min(wz + STEP, LIMIT);  
                else if (c == 'e' || c == 'E') wz = std::max(wz - STEP, -LIMIT); 
                else if (c == ' ') { 
                    vx = 0.0; vy = 0.0; wz = 0.0; 
                }

                if (std::abs(vx) < 0.01) vx = 0.0;
                if (std::abs(vy) < 0.01) vy = 0.0;
                if (std::abs(wz) < 0.01) wz = 0.0;

                target_vx_flu_.store(vx);
                target_vy_flu_.store(vy); // 绝对不碰你的 Vy
                target_wz_flu_.store(wz);

                printf("\r\033[K\033[1;35m[按键 %c]\033[0m -> \033[1;36mVx:\033[0m %5.2f | \033[1;36mVy:\033[0m %5.2f | \033[1;36mWz:\033[0m %5.2f", 
                       (c == ' ' ? 'S' : toupper(c)), vx, vy, wz);
                fflush(stdout);
            }
            std::this_thread::sleep_for(10ms);
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
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
            case BridgeState::INIT_HEARTBEAT:
            case BridgeState::ARM_AND_SWITCH: {
                setpoint_msg.velocity = {0.0f, 0.0f, 0.0f};
                // 【核心修改点】赋 NAN 彻底断开角度闭环，飞控不再纠正偏航角
                setpoint_msg.yaw = NAN;           
                setpoint_msg.yawspeed = 0.0f;     
                
                if (bridge_state_ == BridgeState::INIT_HEARTBEAT) {
                    if (++init_counter_ >= 20) bridge_state_ = BridgeState::ARM_AND_SWITCH;
                } else {
                    if (is_armed && is_offboard) {
                        bridge_state_ = BridgeState::ACTIVE;
                        printf("\n\033[1;32m[INFO] 解锁成功！可以起飞！\033[0m\n");
                    } else {
                        if ((now.nanoseconds() - last_cmd_send_time_ns_.load()) > 1e9) {
                            if (!is_offboard) publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                            if (is_offboard && !is_armed) publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
                            last_cmd_send_time_ns_.store(now.nanoseconds());
                        }
                    }
                }
                break;
            }
            case BridgeState::ACTIVE: {
                // 平滑过渡
                const double alpha = 0.3; 
                smooth_vx_ = alpha * target_vx_flu_.load() + (1.0 - alpha) * smooth_vx_;
                smooth_vy_ = alpha * target_vy_flu_.load() + (1.0 - alpha) * smooth_vy_;
                smooth_wz_ = alpha * target_wz_flu_.load() + (1.0 - alpha) * smooth_wz_;

                auto apply_deadzone = [](double val) -> double { return (std::abs(val) < 0.01) ? 0.0 : val; };
                double vx = apply_deadzone(smooth_vx_);      
                double vy = apply_deadzone(smooth_vy_);      
                double wz = apply_deadzone(smooth_wz_);      

                double current_yaw = current_yaw_ned_.load();

                // 【必须存在的数学转换】：为了确保你按 W 永远是“往机头方向开”，按 A/D 永远是左右平移。
                // 否则飞控会把 W 认作“往地球北极开”。
                setpoint_msg.velocity[0] = vx * cos(current_yaw) + vy * sin(current_yaw); 
                setpoint_msg.velocity[1] = vx * sin(current_yaw) - vy * cos(current_yaw); 
                setpoint_msg.velocity[2] = 0.0;
                
                // 【完全按你的要求修改】
                // 1. 角度赋为 NAN，彻底去掉偏航角闭环，爱朝哪朝哪，飞控不干涉
                setpoint_msg.yaw = NAN;
                // 2. 纯给角速度，按 Q/E 直接控制飞控转弯速率
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
    double smooth_vy_{0.0};
    double smooth_wz_{0.0};
    
    std::atomic<int64_t> last_cmd_send_time_ns_;
    std::atomic<double> current_yaw_ned_;
    std::atomic<double> target_vx_flu_;
    std::atomic<double> target_vy_flu_;
    std::atomic<double> target_wz_flu_;
    std::atomic<uint8_t> nav_state_{0};
    std::atomic<uint8_t> arming_state_{0};

    std::thread keyboard_thread_;
    std::atomic<bool> keep_running_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UsvOffboardTest>());
    rclcpp::shutdown();
    return 0;
}