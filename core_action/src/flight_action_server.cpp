#include "core_action/flight_action_server.hpp"
#include "core_action/target.hpp"
#include <cmath>

FlightActionServer::FlightActionServer(const rclcpp::NodeOptions & options)
    : Node("flight_action_server", options)
{
    using namespace std::placeholders;

    // 创建 Action Server
    this->action_server_ = rclcpp_action::create_server<NavigateToTarget>(
        this,
        "navigate_to_target",
        std::bind(&FlightActionServer::handle_goal, this, _1, _2),
        std::bind(&FlightActionServer::handle_cancel, this, _1),
        std::bind(&FlightActionServer::handle_accepted, this, _1));

    // 创建发布者：发布目标位置
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/setpoint_position/local", 10);

    // 创建订阅者：接收当前位置
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/mavros/local_position/odom", 10,
        std::bind(&FlightActionServer::odom_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Flight Action 服务器 已启动");
}

// 收到目标请求回调
rclcpp_action::GoalResponse FlightActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToTarget::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), 
        "Received goal request: target=(%.2f, %.2f, %.2f, %.2f)",
        goal->x, goal->y, goal->z, goal->yaw);
    
    // 检查当前位置是否有效
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!current_pose_.valid) {
        RCLCPP_WARN(this->get_logger(), "Current position not available yet");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 收到取消请求回调
rclcpp_action::CancelResponse FlightActionServer::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "收到取消目标请求，正在取消...");
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 收到目标被接受回调
void FlightActionServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    // 在新线程中执行具体导航任务，避免阻塞
    std::thread{std::bind(&FlightActionServer::execute, this, goal_handle)}.detach();
}

// 执行导航任务
void FlightActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "开始执行导航任务...");
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateToTarget::Feedback>();
    auto result = std::make_shared<NavigateToTarget::Result>();
    
    float timeout_sec = goal->timeout_sec > 0 ? goal->timeout_sec : 10.0f;
    float stable_time_sec = goal->stable_time_sec > 0 ? goal->stable_time_sec : 0.5f;
    float tolerance = goal->tolerance > 0 ? goal->tolerance : 0.1f;
    
    // 使用 Target 类构造目标
    core_action::Target target(goal->x, goal->y, goal->z, goal->yaw);
    
    // 控制循环频率 20Hz
    rclcpp::Rate rate(20);
    
    auto start_time = this->now();
    int stable_count = 0;
    const int required_stable_frames = static_cast<int>(stable_time_sec * 20);

    while (rclcpp::ok()) {
        // 检查是否被取消
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Goal canceled";
            
            std::lock_guard<std::mutex> lock(pose_mutex_);
            float dx = current_pose_.x - target.get_x();
            float dy = current_pose_.y - target.get_y();
            float dz = current_pose_.z - target.get_z();
            result->final_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "当前目标已取消");
            return;
        }

        // 超时检查
        auto elapsed = (this->now() - start_time).seconds();
        if (elapsed > timeout_sec) {
            result->success = false;
            result->message = "Timeout";
            
            std::lock_guard<std::mutex> lock(pose_mutex_);
            float dx = current_pose_.x - target.get_x();
            float dy = current_pose_.y - target.get_y();
            float dz = current_pose_.z - target.get_z();
            result->final_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            result->total_time = elapsed;
            
            goal_handle->succeed(result);
            RCLCPP_WARN(this->get_logger(), "导航超时!");
            return;
        }

        // 发布目标位置（使用 Target 类）
        auto target_pose = target.get_pose();
        target_pose.header.stamp = this->now();
        pose_pub_->publish(target_pose);

        // 检查是否到达目标
        bool reached = position_reached(target.get_x(), target.get_y(), target.get_z(), target.get_yaw(), tolerance);
        
        if (reached) {
            stable_count++;
        } else {
            stable_count = 0;
        }

        // 发布反馈
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            float dx = current_pose_.x - target.get_x();
            float dy = current_pose_.y - target.get_y();
            float dz = current_pose_.z - target.get_z();
            feedback->distance_remaining = std::sqrt(dx*dx + dy*dy + dz*dz);
            feedback->current_x = current_pose_.x;
            feedback->current_y = current_pose_.y;
            feedback->current_z = current_pose_.z;
            feedback->current_yaw = current_pose_.yaw;
            feedback->elapsed_time = elapsed;
            feedback->stable_count = stable_count;
        }
        goal_handle->publish_feedback(feedback);

        // 检查是否稳定到达
        if (stable_count >= required_stable_frames) {
            result->success = true;
            result->message = "Target reached successfully";
            
            std::lock_guard<std::mutex> lock(pose_mutex_);
            float dx = current_pose_.x - target.get_x();
            float dy = current_pose_.y - target.get_y();
            float dz = current_pose_.z - target.get_z();
            result->final_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            result->total_time = elapsed;
            
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "已成功到达目标点!");
            
            // 更新 Target 的 reached 状态
            target.reached = true;
            return;
        }

        rate.sleep();
    }
}

// 位置检查
bool FlightActionServer::position_reached(
    float target_x, float target_y, float target_z, float target_yaw,
    float tolerance) const
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    
    if (!current_pose_.valid) {
        return false;
    }

    float dx = current_pose_.x - target_x;
    float dy = current_pose_.y - target_y;
    float dz = current_pose_.z - target_z;
    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    float yaw_error = std::abs(current_pose_.yaw - target_yaw);
    // 处理角度wrap-around
    if (yaw_error > M_PI) {
        yaw_error = 2 * M_PI - yaw_error;
    }
    
    return (dist < tolerance) && (yaw_error < 0.1);
}

// 里程计回调
void FlightActionServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    current_pose_.z = msg->pose.pose.position.z;
    
    // 从四元数计算 yaw
    float qz = msg->pose.pose.orientation.z;
    float qw = msg->pose.pose.orientation.w;
    current_pose_.yaw = std::atan2(2.0 * qz * qw, 1.0 - 2.0 * qz * qz);
    if (current_pose_.yaw < 0) {
        current_pose_.yaw += 2 * M_PI;
    }
    
    current_pose_.valid = true;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlightActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
