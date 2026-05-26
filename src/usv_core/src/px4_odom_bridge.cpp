// PX4 里程计转换桥：NED/FRD → ROS2 ENU/FLU，发布标准odom消息与TF变换
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class Px4OdomBridge : public rclcpp::Node
{
public:
    // 节点初始化：创建订阅者、发布者与TF广播器
    Px4OdomBridge() : Node("px4_odom_bridge")
    {
        px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", 
            rclcpp::QoS(10).best_effort(),
            std::bind(&Px4OdomBridge::odomCallback, this, std::placeholders::_1)
        );

        ros_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "\033[1;32m[INIT] PX4_Odom_Bridge started successfully \033[0m");
        RCLCPP_INFO(this->get_logger(), "Translating PX4(NED/FRD) -> ROS2(ENU/FLU)");
    }

    // 节点析构：资源清理
    ~Px4OdomBridge()
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;33m[SHUTDOWN] Detect Ctrl+C, start resource cleanup \033[0m");
        px4_odom_sub_.reset();
        ros_odom_pub_.reset();
        tf_broadcaster_.reset();
        RCLCPP_INFO(this->get_logger(), "Cleanup complete");
    }

private:
    // PX4里程计消息回调：执行坐标系转换并发布ROS2消息
    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // 过滤无效NaN数据
        if (std::isnan(msg->position[0]) || std::isnan(msg->q[0])) {
            return;
        }

        nav_msgs::msg::Odometry odom_msg;
        geometry_msgs::msg::TransformStamped t;
        rclcpp::Time now = this->get_clock()->now();

        // 消息头与坐标系配置
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";      
        odom_msg.child_frame_id = "base_link";  
        
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        // 位置转换：NED(x,y) → ENU(y,x)，Z轴固定抑制漂移
        odom_msg.pose.pose.position.x = msg->position[1];  
        odom_msg.pose.pose.position.y = msg->position[0];  
        odom_msg.pose.pose.position.z = 0.15;
        
        t.transform.translation.x = msg->position[1];
        t.transform.translation.y = msg->position[0];
        t.transform.translation.z = 0.15;

        // 姿态转换：FRD四元数 → FLU RPY → ENU yaw修正
        tf2::Quaternion q_px4(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
        double roll_frd, pitch_frd, yaw_ned;
        tf2::Matrix3x3(q_px4).getRPY(roll_frd, pitch_frd, yaw_ned);

        double roll_flu = roll_frd;
        double pitch_flu = -pitch_frd;
        double yaw_enu = M_PI_2 - yaw_ned;

        // yaw角归一化到[-π, π]范围
        while (yaw_enu > M_PI) yaw_enu -= 2.0 * M_PI;
        while (yaw_enu < -M_PI) yaw_enu += 2.0 * M_PI;

        // 转换回ROS2四元数格式
        tf2::Quaternion q_ros;
        q_ros.setRPY(roll_flu, pitch_flu, yaw_enu);
        
        odom_msg.pose.pose.orientation.x = q_ros.x();
        odom_msg.pose.pose.orientation.y = q_ros.y();
        odom_msg.pose.pose.orientation.z = q_ros.z();
        odom_msg.pose.pose.orientation.w = q_ros.w();

        t.transform.rotation.x = q_ros.x();
        t.transform.rotation.y = q_ros.y();
        t.transform.rotation.z = q_ros.z();
        t.transform.rotation.w = q_ros.w();

        // 线速度转换：分NED帧和BODY_FRD帧两种情况
        if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED) {
            double v_north = msg->velocity[0];
            double v_east = msg->velocity[1];

            // NED速度 → 机体前向/右向速度 → FLU坐标系
            double v_forward = v_north * cos(yaw_ned) + v_east * sin(yaw_ned);
            double v_right = v_east * cos(yaw_ned) - v_north * sin(yaw_ned);
            
            odom_msg.twist.twist.linear.x = v_forward;
            odom_msg.twist.twist.linear.y = -v_right;
            odom_msg.twist.twist.linear.z = 0.00;

            // 速度协方差矩阵旋转转换
            double var_north = msg->velocity_variance[0];
            double var_east = msg->velocity_variance[1];
            double c = cos(yaw_ned);
            double s = sin(yaw_ned);

            odom_msg.twist.covariance[0] = var_north * c * c + var_east * s * s; 
            odom_msg.twist.covariance[7] = var_east * c * c + var_north * s * s; 
            odom_msg.twist.covariance[14] = 1e-6;
        } 
        else if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
            // BODY_FRD → FLU直接转换
            odom_msg.twist.twist.linear.x = msg->velocity[0];
            odom_msg.twist.twist.linear.y = -msg->velocity[1];
            odom_msg.twist.twist.linear.z = 0.00;

            odom_msg.twist.covariance[0] = msg->velocity_variance[0];
            odom_msg.twist.covariance[7] = msg->velocity_variance[1];
            odom_msg.twist.covariance[14] = 1e-6;
        }

        // 角速度转换：FRD → FLU
        odom_msg.twist.twist.angular.x = msg->angular_velocity[0];
        odom_msg.twist.twist.angular.y = -msg->angular_velocity[1];
        odom_msg.twist.twist.angular.z = -msg->angular_velocity[2];

        // 角速度协方差（经验值）
        odom_msg.twist.covariance[21] = 0.01;
        odom_msg.twist.covariance[28] = 0.01;
        odom_msg.twist.covariance[35] = 0.01;

        // 位姿协方差转换
        odom_msg.pose.covariance[0] = msg->position_variance[1];
        odom_msg.pose.covariance[7] = msg->position_variance[0];
        odom_msg.pose.covariance[14] = 1e-6;
        odom_msg.pose.covariance[21] = msg->orientation_variance[0]; 
        odom_msg.pose.covariance[28] = msg->orientation_variance[1]; 
        odom_msg.pose.covariance[35] = msg->orientation_variance[2]; 

        // 发布ROS2标准里程计与TF变换
        ros_odom_pub_->publish(odom_msg);
        tf_broadcaster_->sendTransform(t);
    }

    // 成员变量
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ros_odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

// 主函数：初始化ROS2并运行节点
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4OdomBridge>());
    rclcpp::shutdown();
    return 0;
}