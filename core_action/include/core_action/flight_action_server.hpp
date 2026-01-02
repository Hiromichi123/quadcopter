#ifndef FLIGHT_ACTION_SERVER_HPP
#define FLIGHT_ACTION_SERVER_HPP

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "core_action/action/navigate_to_target.hpp"

class FlightActionServer : public rclcpp::Node {
public:
    using NavigateToTarget = core_action::action::NavigateToTarget;
    using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToTarget>;

    explicit FlightActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // Action Server 回调函数
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToTarget::Goal> goal);
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle);
    
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

    // 执行导航任务（独立线程）
    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    // 位置检查
    bool position_reached(
        float target_x, float target_y, float target_z, float target_yaw,
        float tolerance) const;

    // Odometry 回调
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // 成员变量
    rclcpp_action::Server<NavigateToTarget>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // 当前位置信息
    struct CurrentPose {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float yaw = 0.0f;
        bool valid = false;
    } current_pose_;

    mutable std::mutex pose_mutex_;
};

#endif // FLIGHT_ACTION_SERVER_HPP
