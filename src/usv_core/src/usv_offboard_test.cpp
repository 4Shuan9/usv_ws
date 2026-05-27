#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/**
 * @brief USV测试状态枚举
 */
enum USVState {
    INIT,           // 初始化与心跳建立
    ARM_AND_HOLD,   // 解锁并保持静止
    DISARM,         // 上锁
    FINISHED        // 测试完成
};

class USVOffboardTest : public rclcpp::Node
{
public:
    USVOffboardTest() : Node("usv_offboard_test")
    {
        // 初始化发布器
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        
        // 状态变量初始化
        offboard_setpoint_counter_ = 0;
        flight_state_ = USVState::INIT;
        state_start_time_ = this->now();
        
        // 100ms定时器 (10Hz)
        auto timer_callback = [this]() -> void 
        {
            auto current_time = this->now();
            auto state_duration = (current_time - state_start_time_).seconds();
            
            switch (flight_state_) 
            {
                case USVState::INIT:
                {
                    // 必须先发送至少一秒钟的设定点(10次)才能进入Offboard模式
                    if (offboard_setpoint_counter_ < 10) 
                    {
                        publish_offboard_control_mode();
                        publish_trajectory_setpoint(0.0f, 0.0f); // 发送0速度
                        offboard_setpoint_counter_++;
                    } 
                else if (offboard_setpoint_counter_ == 10) 
                {
                    // 加上这两句，保证数据流不断
                    publish_offboard_control_mode();
                    publish_trajectory_setpoint(0.0f, 0.0f);

                    RCLCPP_INFO(this->get_logger(), "Switching to OFFBOARD mode...");
                    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                    
                    RCLCPP_INFO(this->get_logger(), "Sending ARM command...");
                    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
                    
                    offboard_setpoint_counter_++;
                    flight_state_ = USVState::ARM_AND_HOLD;
                    state_start_time_ = this->now();
                }
                    break;
                }
                
                case USVState::ARM_AND_HOLD:
                {
                    // 持续发送设定点保持Offboard模式
                    publish_offboard_control_mode();
                    publish_trajectory_setpoint(0.0f, 0.0f); // 保持0速度静止
                    
                    if (state_duration > 10.0) // 保持解锁状态10秒钟用于观察
                    {
                        RCLCPP_INFO(this->get_logger(), "Holding complete. Starting disarm process.");
                        flight_state_ = USVState::DISARM;
                        state_start_time_ = this->now();
                    }
                    break;
                }
                
                case USVState::DISARM:
                {
                    if (state_duration < 1.0) 
                    {
                        publish_offboard_control_mode();
                        publish_trajectory_setpoint(0.0f, 0.0f);
                        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "Disarmed. Test mission complete!!!");
                        flight_state_ = USVState::FINISHED;
                    }
                    break;
                }
                
                case USVState::FINISHED:
                {
                    RCLCPP_INFO_ONCE(this->get_logger(), "Press Ctrl+C to exit.");
                    break;
                }
            }
        };
        
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    
    uint64_t offboard_setpoint_counter_;
    USVState flight_state_;
    rclcpp::Time state_start_time_;
    
    /**
     * @brief 发布离线控制模式 (修改为速度控制)
     */
    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = false;      // USV测试中关闭位置控制
        msg.velocity = true;       // 开启速度控制
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = 0;
        offboard_control_mode_publisher_->publish(msg);
    }
    
    /**
     * @brief 发布轨迹设定点 (基于速度)
     */
    void publish_trajectory_setpoint(float vel_x, float vel_y)
    {
        TrajectorySetpoint msg{};
        // 位置设定为 NaN，告诉飞控我们不关心具体位置
        msg.position = {std::numeric_limits<float>::quiet_NaN(), 
                        std::numeric_limits<float>::quiet_NaN(), 
                        std::numeric_limits<float>::quiet_NaN()};
        
        // 设定机体坐标系下的速度 (前向 vel_x, 侧向 vel_y, Z轴速度为0)
        msg.velocity = {vel_x, vel_y, 0.0f};
        
        // 偏航角和偏航角速度也设为 NaN
        msg.yaw = std::numeric_limits<float>::quiet_NaN();
        msg.yawspeed = std::numeric_limits<float>::quiet_NaN();
        
        msg.timestamp = 0;
        trajectory_setpoint_publisher_->publish(msg);
    }
    
    /**
     * @brief 发布载具指令
     */
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting USV offboard arming test node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<USVOffboardTest>());
    rclcpp::shutdown();
    return 0;
}