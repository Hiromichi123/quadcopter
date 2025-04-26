#ifndef TARGET_HPP
#define TARGET_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

// 目标点类
class target {
public:
    bool reached;

    target() : reached(false) {}
    
    target(float x, float y, float z, float yaw) : reached(false) {
        pose_stamped.header.stamp = rclcpp::Clock().now(); // 时间戳
        pose_stamped.header.frame_id = "map"; // map代表全局坐标系
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.position.z = z;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = std::sin(yaw / 2.0);
        pose_stamped.pose.orientation.w = std::cos(yaw / 2.0);
    }

    // 获取成员
    float get_x() const { return pose_stamped.pose.position.x; }
    float get_y() const { return pose_stamped.pose.position.y; }
    float get_z() const { return pose_stamped.pose.position.z; }
    float get_yaw() const { float yaw = std::atan2(2.0 * (pose_stamped.pose.orientation.z * pose_stamped.pose.orientation.w), 
                                        1.0 - 2.0 * (pose_stamped.pose.orientation.z * pose_stamped.pose.orientation.z));
                            if (yaw < 0) yaw += 2 * M_PI;
                            return yaw; }
    
    // 重要：获取发布所需的PoseStamped结构体
    geometry_msgs::msg::PoseStamped get_pose() { return pose_stamped; }

    // 设置成员
    void set_x(float x) { pose_stamped.pose.position.x = x; }
    void set_y(float y) { pose_stamped.pose.position.y = y; }
    void set_z(float z) { pose_stamped.pose.position.z = z; }
    void set_yaw(float yaw) {
        pose_stamped.pose.orientation.z = std::sin(yaw / 2.0);
        pose_stamped.pose.orientation.w = std::cos(yaw / 2.0);
    }

    // 设置时间戳
    void set_time(rclcpp::Time time) { pose_stamped.header.stamp = time; }

    // 允许 target 直接转换为 PoseStamped
    operator geometry_msgs::msg::PoseStamped&() { return pose_stamped; }
    operator const geometry_msgs::msg::PoseStamped&() const { return pose_stamped; }

private:
    geometry_msgs::msg::PoseStamped pose_stamped;
};
#endif
