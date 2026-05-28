#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <cmath>

class Rf2oToPx4 : public rclcpp::Node
{
public:
    Rf2oToPx4() : Node("rf2o_to_px4")
    {
        rf2o_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_rf2o", 10,
            std::bind(&Rf2oToPx4::odomCallback, this, std::placeholders::_1)
        );

        px4_visual_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10
        );

        RCLCPP_INFO(this->get_logger(), "\033[1;32m[INIT] RF2O to PX4 Bridge started (Compass-Friendly Edition)\033[0m");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        px4_msgs::msg::VehicleOdometry px4_odom;
        
        px4_odom.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        px4_odom.timestamp_sample = px4_odom.timestamp;

        // 位置是全局坐标系 (NED)
        px4_odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        // 速度是机体坐标系 (BODY_FRD)
        px4_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;

        // --- 位置转换 (ROS ENU -> PX4 NED) ---
        px4_odom.position[0] = msg->pose.pose.position.y;  // North
        px4_odom.position[1] = msg->pose.pose.position.x;  // East
        px4_odom.position[2] = 0.0;                        // Z强制清零
        
        // 稍微放宽一点位置方差，让 EKF2 融合更平滑
        px4_odom.position_variance = {0.05f, 0.05f, 0.05f}; 

        // --- 姿态隔离 (关键修复!) ---
        // 强行写入 NAN，告诉 EKF2："我没有姿态数据，请完全信任你自带的磁罗盘！"
        px4_odom.q[0] = NAN;
        px4_odom.q[1] = NAN;
        px4_odom.q[2] = NAN;
        px4_odom.q[3] = NAN;
        px4_odom.orientation_variance = {NAN, NAN, NAN};

        // --- 速度转换 (FLU -> FRD) ---
        px4_odom.velocity[0] = msg->twist.twist.linear.x;  // Forward
        px4_odom.velocity[1] = -msg->twist.twist.linear.y; // Right (FLU的左为正，FRD的右为正，所以取反)
        px4_odom.velocity[2] = 0.0;
        px4_odom.velocity_variance = {0.05f, 0.05f, 0.05f};

        // --- 角速度转换 (FLU -> FRD) ---
        px4_odom.angular_velocity[0] = msg->twist.twist.angular.x;
        px4_odom.angular_velocity[1] = -msg->twist.twist.angular.y;
        px4_odom.angular_velocity[2] = -msg->twist.twist.angular.z;

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