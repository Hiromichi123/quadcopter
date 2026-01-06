#ifndef CORE_ACTION_TARGET_HPP
#define CORE_ACTION_TARGET_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace core_action {
    // 目标点类
    class Target {
    public:
        mutable bool reached; // 是否到达目标点

        Target() : reached(false) {}
        
        Target(float x, float y, float z, float yaw = 0.0f) : reached(false) {
            pose_stamped.header.stamp = rclcpp::Clock().now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = x;
            pose_stamped.pose.position.y = y;
            pose_stamped.pose.position.z = z;
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = std::sin(yaw / 2.0);
            pose_stamped.pose.orientation.w = std::cos(yaw / 2.0);
        }

        // 从 PoseStamped 快速构造，必须显式调用
        explicit Target(const geometry_msgs::msg::PoseStamped& pose) : reached(false) {
            pose_stamped = pose;
        }

        // 获取成员
        float get_x() const { return pose_stamped.pose.position.x; }
        float get_y() const { return pose_stamped.pose.position.y; }
        float get_z() const { return pose_stamped.pose.position.z; }
        float get_yaw() const { 
            float yaw = std::atan2(
                2.0 * (pose_stamped.pose.orientation.z * pose_stamped.pose.orientation.w), 
                1.0 - 2.0 * (pose_stamped.pose.orientation.z * pose_stamped.pose.orientation.z)
            );
            return (yaw < 0)? yaw + 2 * M_PI : yaw;
        }
        
        // 获取发布所需的PoseStamped结构体
        auto get_pose() const { return pose_stamped; }

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

        // 手动/隐式转换
        auto to_pose() const -> geometry_msgs::msg::PoseStamped { return pose_stamped; }
        operator geometry_msgs::msg::PoseStamped&() { return pose_stamped; }

    private:
        geometry_msgs::msg::PoseStamped pose_stamped;
    };
}

#endif // CORE_ACTION_TARGET_HPP
