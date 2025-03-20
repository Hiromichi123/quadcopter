#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ros2_tools/msg/lidar_pose.hpp"

class LidarDataNode : public rclcpp::Node {
public:
    bool using_gazebo = false; // TODO:最好改为启动参数

    LidarDataNode() : Node("lidar_data_node") {
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        // LidarPose 发布
        lidar_pub = this->create_publisher<ros2_tools::msg::LidarPose>("lidar_data", 10);
        RCLCPP_INFO(this->get_logger(), "lidar_data publisher created");

        // Odometry 订阅
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/Odometry", 10, std::bind(&LidarDataNode::odomCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Odometry subscription successful.");

        // px4_local_position 订阅
        local_position_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", qos_profile, std::bind(&LidarDataNode::localPositionCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "PX4 Local Position subscription successful.");
    }

    // 雷达数据回调
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!using_gazebo) msgDispose(msg->pose.pose); // 实机模式，处理雷达数据
    }

    // PX4数据回调
    void localPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (using_gazebo) msgDispose(msg->pose); // 仿真模式，处理PX4数据
    }

    // msg统一处理函数
    void msgDispose(const geometry_msgs::msg::Pose &pose) {
        // 转换四元数为欧拉角
        double x = pose.position.x;
        double y = pose.position.y;
        double z = pose.position.z;
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 确保欧拉角为正
        if (roll < 0) roll += 2 * M_PI;
        if (pitch < 0) pitch += 2 * M_PI;
        if (yaw < 0) yaw += 2 * M_PI;

        // 填充LidarPose消息
        lidar_pose.x = x;
        lidar_pose.y = y;
        lidar_pose.z = z;
        lidar_pose.roll = roll;
        lidar_pose.pitch = pitch;
        lidar_pose.yaw = yaw;

        lidar_pub->publish(lidar_pose);

        static int counter = 0;
        if (++counter >= 50) {
            RCLCPP_INFO(this->get_logger(), 
                        "Position=(%.2f, %.2f, %.2f), Orientation=(%.2f, %.2f, %.2f) rad",
                        x, y, z, roll, pitch, yaw);
            counter = 0;
        }
    }

private:
    rclcpp::Publisher<ros2_tools::msg::LidarPose>::SharedPtr lidar_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub;

    ros2_tools::msg::LidarPose lidar_pose;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarDataNode>();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lidar node completed");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}