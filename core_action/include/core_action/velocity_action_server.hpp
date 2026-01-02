#ifndef VELOCITY_ACTION_SERVER_HPP
#define VELOCITY_ACTION_SERVER_HPP

#include <memory>
#include <thread>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "core_action/action/fly_by_velocity.hpp"

class VelocityActionServer : public rclcpp::Node {
public:
    using FlyByVelocity = core_action::action::FlyByVelocity;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FlyByVelocity>;

    explicit VelocityActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // Action Server 回调函数
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FlyByVelocity::Goal> goal);
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle);
    
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

    // 执行速度控制任务（独立线程）
    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    // Odometry 回调
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // 成员变量
    rclcpp_action::Server<FlyByVelocity>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // 当前位置和速度信息
    struct CurrentState {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float vx = 0.0f;
        float vy = 0.0f;
        float vz = 0.0f;
        bool valid = false;
    } current_state_;

    mutable std::mutex state_mutex_;
};

#endif // VELOCITY_ACTION_SERVER_HPP
