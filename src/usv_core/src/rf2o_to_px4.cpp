#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class Rf2oToPx4 : public rclcpp::Node
{
public:
    Rf2oToPx4() : Node("rf2o_to_px4")
    {
        // 订阅 rf2o 输出的里程计
        rf2o_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_rf2o", 10,
            std::bind(&Rf2oToPx4::odomCallback, this, std::placeholders::_1)
        );

        // 发布给 PX4 的外部视觉里程计话题
        px4_visual_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10
        );

        RCLCPP_INFO(this->get_logger(), "\033[1;32m[INIT] RF2O to PX4 Bridge started \033[0m");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        px4_msgs::msg::VehicleOdometry px4_odom;
        
        // 必须使用系统时间，与 MicroXRCEAgent 时间同步
        px4_odom.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        px4_odom.timestamp_sample = px4_odom.timestamp;

        // 设置参考系为 NED (PX4标准)
        px4_odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        px4_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

        // --- 位置转换 (ROS ENU -> PX4 NED) ---
        // ROS: X=东, Y=北, Z=天 | PX4: X=北, Y=东, Z=地
        px4_odom.position[0] = msg->pose.pose.position.y;  // North = Y
        px4_odom.position[1] = msg->pose.pose.position.x;  // East = X
        px4_odom.position[2] = 0.0;                        // USV 强制 Z=0，防止高度漂移

        // --- 姿态转换 (ENU -> NED) ---
        tf2::Quaternion q_enu(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        double roll_enu, pitch_enu, yaw_enu;
        tf2::Matrix3x3(q_enu).getRPY(roll_enu, pitch_enu, yaw_enu);

        // 偏航角转换 (NED Yaw = Pi/2 - ENU Yaw)
        double yaw_ned = M_PI_2 - yaw_enu;
        while (yaw_ned > M_PI) yaw_ned -= 2.0 * M_PI;
        while (yaw_ned < -M_PI) yaw_ned += 2.0 * M_PI;

        tf2::Quaternion q_ned;
        q_ned.setRPY(roll_enu, -pitch_enu, yaw_ned);
        
        // PX4 四元数顺序是 w, x, y, z
        px4_odom.q[0] = q_ned.w();
        px4_odom.q[1] = q_ned.x();
        px4_odom.q[2] = q_ned.y();
        px4_odom.q[3] = q_ned.z();

        // --- 速度转换 (FLU -> FRD) ---
        // 机体系下：X前，Y左，Z上 (FLU) -> X前，Y右，Z下 (FRD)
        px4_odom.velocity[0] = msg->twist.twist.linear.x;
        px4_odom.velocity[1] = -msg->twist.twist.linear.y;
        px4_odom.velocity[2] = 0.0;

        px4_odom.angular_velocity[0] = msg->twist.twist.angular.x;
        px4_odom.angular_velocity[1] = -msg->twist.twist.angular.y;
        px4_odom.angular_velocity[2] = -msg->twist.twist.angular.z;

        // 设置方差 (告诉 EKF2 相信这份数据的程度，数值越小越信任)
        px4_odom.position_variance = {0.01f, 0.01f, 0.01f};
        px4_odom.orientation_variance = {0.01f, 0.01f, 0.01f};
        px4_odom.velocity_variance = {0.01f, 0.01f, 0.01f};

        px4_visual_odom_pub_->publish(px4_odom);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rf2o_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_visual_odom_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Rf2oToPx4>());
    rclcpp::shutdown();
    return 0;
}