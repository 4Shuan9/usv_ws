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
    INIT_HEARTBEAT, // 初始发送心跳，建立连接
    ARM_AND_SWITCH, // 解锁并切换至 Offboard 模式
    ACTIVE          // 正常控制态
};

class UsvOffboardTest : public rclcpp::Node
{
public:
    UsvOffboardTest() : Node("usv_offboard_test"), 
        current_yaw_ned_(0.0), target_vx_flu_(0.0), target_vy_flu_(0.0), target_wz_flu_(0.0)
    {
        // ----------------- 订阅器 -----------------
        px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            std::bind(&UsvOffboardTest::odomCallback, this, std::placeholders::_1));

        px4_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", rclcpp::QoS(10).best_effort(),
            std::bind(&UsvOffboardTest::statusCallback, this, std::placeholders::_1));

        // ----------------- 发布器 -----------------
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // ----------------- 状态初始化 -----------------
        bridge_state_ = BridgeState::INIT_HEARTBEAT;
        init_counter_ = 0;
        last_cmd_send_time_ns_.store(this->get_clock()->now().nanoseconds());

        // 控制主循环，50ms (20Hz)
        control_timer_ = this->create_wall_timer(50ms, std::bind(&UsvOffboardTest::controlLoop, this));

        // ----------------- 终端 UI 打印 -----------------
        printWelcomeMenu();

        // ----------------- 启动键盘监听线程 -----------------
        keep_running_ = true;
        keyboard_thread_ = std::thread(&UsvOffboardTest::keyboardLoop, this);
    }

    ~UsvOffboardTest() {
        keep_running_ = false;
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        printf("\n\033[1;31m[INFO] USV Offboard 控制节点已退出。\033[0m\n");
    }

private:
    void printWelcomeMenu() {
        RCLCPP_INFO(this->get_logger(), "\033[1;36m========================================================\033[0m");
        RCLCPP_INFO(this->get_logger(), "\033[1;32m          🚀 USV Offboard 键盘控制节点已启动 🚀          \033[0m");
        RCLCPP_INFO(this->get_logger(), "\033[1;36m========================================================\033[0m");
        RCLCPP_INFO(this->get_logger(), " [\033[1;33m W / S \033[0m] : 前后平移 (机体X轴)  ±0.05 m/s (限幅 ±5.0)");
        RCLCPP_INFO(this->get_logger(), " [\033[1;33m A / D \033[0m] : 左右平移 (机体Y轴)  ±0.05 m/s (限幅 ±5.0)");
        RCLCPP_INFO(this->get_logger(), " [\033[1;33m Q / E \033[0m] : 左右转弯 (Yaw轴)    ±0.05 rad/s(限幅 ±5.0)");
        RCLCPP_INFO(this->get_logger(), " [\033[1;31m 空 格 \033[0m] : 紧急悬停 (所有指令立刻归零)");
        RCLCPP_INFO(this->get_logger(), "\033[1;36m========================================================\033[0m");
        printf("\n等待按键输入...\n");
    }

    void keyboardLoop() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        
        // 关闭标准输入的回显和行缓冲
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        
        // 设置非阻塞读取
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

        const double STEP = 0.05;
        const double LIMIT = 5.0;

        while (keep_running_ && rclcpp::ok()) {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0) {
                double vx = target_vx_flu_.load();
                double vy = target_vy_flu_.load();
                double wz = target_wz_flu_.load();

                // 解析按键，加入限幅
                if      (c == 'w' || c == 'W') vx = std::min(vx + STEP, LIMIT);
                else if (c == 's' || c == 'S') vx = std::max(vx - STEP, -LIMIT);
                else if (c == 'a' || c == 'A') vy = std::min(vy + STEP, LIMIT);  // 左平移
                else if (c == 'd' || c == 'D') vy = std::max(vy - STEP, -LIMIT); // 右平移
                else if (c == 'q' || c == 'Q') wz = std::min(wz + STEP, LIMIT);  // 左转 (CCW)
                else if (c == 'e' || c == 'E') wz = std::max(wz - STEP, -LIMIT); // 右转 (CW)
                else if (c == ' ') { 
                    vx = 0.0; vy = 0.0; wz = 0.0; 
                }

                // 浮点数消抖：极其靠近0时直接归零，防止出现 0.000001 之类的显示
                if (std::abs(vx) < 0.01) vx = 0.0;
                if (std::abs(vy) < 0.01) vy = 0.0;
                if (std::abs(wz) < 0.01) wz = 0.0;

                target_vx_flu_.store(vx);
                target_vy_flu_.store(vy);
                target_wz_flu_.store(wz);

                // 优雅的终端单行刷新打印
                printf("\r\033[K\033[1;35m[按键 %c]\033[0m 目标指令 -> \033[1;36mVx:\033[0m %5.2f m/s  |  \033[1;36mVy:\033[0m %5.2f m/s  |  \033[1;36mWz:\033[0m %5.2f rad/s", 
                       (c == ' ' ? 'S' : toupper(c)), vx, vy, wz);
                fflush(stdout);
            }
            std::this_thread::sleep_for(10ms);
        }

        // 恢复终端设置
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

    void controlLoop() {
        auto now = this->get_clock()->now();
        uint64_t timestamp = now.nanoseconds() / 1000; 

        // 1. 发送 Offboard 心跳
        px4_msgs::msg::OffboardControlMode mode_msg;
        mode_msg.timestamp = timestamp;
        mode_msg.velocity = true;
        offboard_control_mode_pub_->publish(mode_msg);

        // 2. 准备控制目标数据
        px4_msgs::msg::TrajectorySetpoint setpoint_msg;
        setpoint_msg.timestamp = timestamp;
        setpoint_msg.position = {NAN, NAN, NAN};
        setpoint_msg.acceleration = {NAN, NAN, NAN};

        bool is_armed = (arming_state_.load() == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
        bool is_offboard = (nav_state_.load() == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);

        // 3. 状态机流转
        switch (bridge_state_) {
            case BridgeState::INIT_HEARTBEAT: {
                setpoint_msg.velocity = {0.0f, 0.0f, 0.0f};
                setpoint_msg.yaw = current_yaw_ned_.load();
                if (++init_counter_ >= 20) {
                    bridge_state_ = BridgeState::ARM_AND_SWITCH;
                }
                break;
            }
            case BridgeState::ARM_AND_SWITCH: {
                setpoint_msg.velocity = {0.0f, 0.0f, 0.0f};
                setpoint_msg.yaw = current_yaw_ned_.load(); // 保持当前偏航角
                
                if (is_armed && is_offboard) {
                    bridge_state_ = BridgeState::ACTIVE;
                    printf("\n\033[1;32m[INFO] 切换 Offboard 成功，系统已就绪，可以开始控制！\033[0m\n");
                } else {
                    int64_t last_send_ns = last_cmd_send_time_ns_.load();
                    // 每秒发送一次切换模式/解锁指令
                    if ((now.nanoseconds() - last_send_ns) > 1e9) {
                        if (!is_offboard) publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                        if (is_offboard && !is_armed) publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
                        last_cmd_send_time_ns_.store(now.nanoseconds());
                    }
                }
                break;
            }
            case BridgeState::ACTIVE: {
                const double alpha = 0.3; // 一阶低通滤波系数
                
                // 简单的平滑处理，防止输入阶跃
                smooth_vx_ = alpha * target_vx_flu_.load() + (1.0 - alpha) * smooth_vx_;
                smooth_vy_ = alpha * target_vy_flu_.load() + (1.0 - alpha) * smooth_vy_;
                smooth_wz_ = alpha * target_wz_flu_.load() + (1.0 - alpha) * smooth_wz_;

                // 死区函数：防止浮点误差导致始终存在微小速度
                auto apply_deadzone = [](double val) -> double {
                    return (std::abs(val) < 0.01) ? 0.0 : val;
                };

                double vx = apply_deadzone(smooth_vx_);      
                double vy = apply_deadzone(smooth_vy_);      
                double wz = apply_deadzone(smooth_wz_);      

                double current_yaw = current_yaw_ned_.load();

                // 核心逻辑：将机体系 FLU (前左上) 速度映射到全局系 NED (北东地)
                // 北向速度 (North)
                setpoint_msg.velocity[0] = vx * cos(current_yaw) + vy * sin(current_yaw); 
                // 东向速度 (East)
                setpoint_msg.velocity[1] = vx * sin(current_yaw) - vy * cos(current_yaw); 
                // 下向速度 (Down) - 保持平面运动
                setpoint_msg.velocity[2] = 0.0;
                
                // 完全去掉外部偏航闭环，交由PX4内部控制
                // 设置 yaw 为 NAN，让其失效
                setpoint_msg.yaw = NAN;
                // 传递角速度指令，注意符号：
                // 机体坐标系FLU左正转(CCW) -> 对应 NED 下方右正转(CW)，所以加负号
                setpoint_msg.yawspeed = -wz; 
                break;
            }
        }
        
        trajectory_setpoint_pub_->publish(setpoint_msg);
    }

    // ----------------- 类成员变量 -----------------
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr px4_status_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    BridgeState bridge_state_;
    int init_counter_;
    
    // 平滑后的执行值
    double smooth_vx_{0.0};
    double smooth_vy_{0.0};
    double smooth_wz_{0.0};
    
    // 原子变量，用于多线程安全(ROS定时器线程 vs 键盘监听线程)
    std::atomic<int64_t> last_cmd_send_time_ns_;
    std::atomic<double> current_yaw_ned_;
    std::atomic<double> target_vx_flu_;
    std::atomic<double> target_vy_flu_;
    std::atomic<double> target_wz_flu_;
    std::atomic<uint8_t> nav_state_{0};
    std::atomic<uint8_t> arming_state_{0};

    // 键盘监听
    std::thread keyboard_thread_;
    std::atomic<bool> keep_running_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UsvOffboardTest>());
    rclcpp::shutdown();
    return 0;
}