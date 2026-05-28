#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class Px4ToNav2 : public rclcpp::Node
{
public:
    Px4ToNav2() : Node("px4_to_nav2")
    {
        px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", 
            rclcpp::QoS(10).best_effort(),
            std::bind(&Px4ToNav2::odomCallback, this, std::placeholders::_1)
        );

        ros_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "\033[1;32m[INIT] PX4_To_Nav2 Bridge started \033[0m");
    }

private:
    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        if (std::isnan(msg->position[0]) || std::isnan(msg->q[0])) return;

        nav_msgs::msg::Odometry odom_msg;
        geometry_msgs::msg::TransformStamped t;
        
        // 关键：强制使用系统时钟同步
        rclcpp::Time now = this->get_clock()->now();

        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";      
        odom_msg.child_frame_id = "base_link";  
        
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = msg->position[1];  
        odom_msg.pose.pose.position.y = msg->position[0];  
        odom_msg.pose.pose.position.z = 0.15;
        
        t.transform.translation.x = msg->position[1];
        t.transform.translation.y = msg->position[0];
        t.transform.translation.z = 0.15;

        tf2::Quaternion q_px4(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
        double roll_frd, pitch_frd, yaw_ned;
        tf2::Matrix3x3(q_px4).getRPY(roll_frd, pitch_frd, yaw_ned);

        double roll_flu = roll_frd;
        double pitch_flu = -pitch_frd;
        double yaw_enu = M_PI_2 - yaw_ned;

        while (yaw_enu > M_PI) yaw_enu -= 2.0 * M_PI;
        while (yaw_enu < -M_PI) yaw_enu += 2.0 * M_PI;

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

        if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED) {
            double v_north = msg->velocity[0];
            double v_east = msg->velocity[1];
            double v_forward = v_north * cos(yaw_ned) + v_east * sin(yaw_ned);
            double v_right = v_east * cos(yaw_ned) - v_north * sin(yaw_ned);
            odom_msg.twist.twist.linear.x = v_forward;
            odom_msg.twist.twist.linear.y = -v_right;
            odom_msg.twist.twist.linear.z = 0.00;
        } else {
            odom_msg.twist.twist.linear.x = msg->velocity[0];
            odom_msg.twist.twist.linear.y = -msg->velocity[1];
            odom_msg.twist.twist.linear.z = 0.00;
        }

        odom_msg.twist.twist.angular.x = msg->angular_velocity[0];
        odom_msg.twist.twist.angular.y = -msg->angular_velocity[1];
        odom_msg.twist.twist.angular.z = -msg->angular_velocity[2];

        // 统一固定协方差
        for (int i=0; i<36; i+=7) {
            odom_msg.pose.covariance[i] = 1e-3;
            odom_msg.twist.covariance[i] = 1e-3;
        }

        ros_odom_pub_->publish(odom_msg);
        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ros_odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4ToNav2>());
    rclcpp::shutdown();
    return 0;
}