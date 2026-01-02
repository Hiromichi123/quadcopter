#include "core_action/velocity_action_server.hpp"
#include "core_action/velocity.hpp"
#include <cmath>
#include <algorithm>

VelocityActionServer::VelocityActionServer(const rclcpp::NodeOptions & options)
    : Node("velocity_action_server", options)
{
    using namespace std::placeholders;

    // 创建 Action Server
    this->action_server_ = rclcpp_action::create_server<FlyByVelocity>(
        this,
        "fly_by_velocity",
        std::bind(&VelocityActionServer::handle_goal, this, _1, _2),
        std::bind(&VelocityActionServer::handle_cancel, this, _1),
        std::bind(&VelocityActionServer::handle_accepted, this, _1));

    // vel发布
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    // 里程计订阅
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/mavros/local_position/odom", 10, std::bind(&VelocityActionServer::odom_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Velocity Action 服务器 已启动");
}

rclcpp_action::GoalResponse VelocityActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FlyByVelocity::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), 
        "收到速度控制请求: vel=(%.2f, %.2f, %.2f) m/s, vyaw=%.2f rad/s, 持续时间=%.2fs",
        goal->vx, goal->vy, goal->vz, goal->vyaw, goal->duration);
    
    // 检查当前状态是否有效
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!current_state_.valid) {
        RCLCPP_WARN(this->get_logger(), "当前状态信息不可用");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // 安全限速
    const float max_velocity = 1.0f;  // 最大 1 m/s
    float speed = std::sqrt(goal->vx * goal->vx + goal->vy * goal->vy + goal->vz * goal->vz);
    if (speed > max_velocity) {
        RCLCPP_WARN(this->get_logger(), "请求速度 %.2f m/s 超过限制 %.2f m/s", speed, max_velocity);
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 收到取消请求回调
rclcpp_action::CancelResponse VelocityActionServer::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "收到取消速度控制请求");
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 收到目标被接受回调
void VelocityActionServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&VelocityActionServer::execute, this, goal_handle)}.detach(); // 在新线程中执行任务，避免阻塞
}

void VelocityActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "开始执行速度控制任务...");
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FlyByVelocity::Feedback>();
    auto result = std::make_shared<FlyByVelocity::Result>();
    
    core_action::Velocity vel_cmd(goal->vx, goal->vy, goal->vz, goal->vyaw); // 构造速度指令
    
    // 控制循环频率 20Hz
    rclcpp::Rate rate(20);
    
    auto start_time = this->now();
    float initial_altitude = 0.0f;
    bool has_initial_altitude = false;
    
    // 获取初始高度，用于定高
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (current_state_.valid) {
            initial_altitude = goal->maintain_altitude ? 
                              (goal->altitude_target > 0 ? goal->altitude_target : current_state_.z) 
                              : current_state_.z;
            has_initial_altitude = true;
        }
    }

    while (rclcpp::ok()) {
        // 检查是否被取消
        if (goal_handle->is_canceling()) {
            // 发送停止指令
            core_action::Velocity stop_vel(0, 0, 0, 0);
            auto twist = stop_vel.get_twist();
            twist.header.stamp = this->now();
            vel_pub_->publish(twist);
            
            result->success = false;
            result->message = "速度控制被取消";
            auto elapsed = (this->now() - start_time).seconds();
            result->actual_duration = elapsed;
            
            std::lock_guard<std::mutex> lock(state_mutex_);
            result->final_altitude = current_state_.z;
            
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "速度控制已取消");
            return;
        }

        // 检查是否达到持续时间
        auto elapsed = (this->now() - start_time).seconds();
        if (goal->duration > 0 && elapsed >= goal->duration) {
            // 发送停止指令
            core_action::Velocity stop_vel(0, 0, 0, 0);
            auto twist = stop_vel.get_twist();
            twist.header.stamp = this->now();
            vel_pub_->publish(twist);
            
            result->success = true;
            result->message = "速度控制完成";
            result->actual_duration = elapsed;
            
            std::lock_guard<std::mutex> lock(state_mutex_);
            result->final_altitude = current_state_.z;
            
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "速度控制任务完成！");
            return;
        }

        // 定高逻辑
        if (goal->maintain_altitude && has_initial_altitude) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            float z_error = initial_altitude - current_state_.z;
            
            // 如果误差超过死区，进行修正
            if (std::abs(z_error) > 0.1f) {
                // P控制，限制最大垂直速度为 0.5 m/s
                float vz_correction = std::clamp(z_error * 1.0f, -0.5f, 0.5f);
                vel_cmd.set_vz(vz_correction);
            } else {
                vel_cmd.set_vz(0.0f); // 死区内保持vz=0
            }
        }

        // 发布速度指令
        auto twist = vel_cmd.get_twist();
        twist.header.stamp = this->now();
        vel_pub_->publish(twist);

        // 发布反馈
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            feedback->current_vx = current_state_.vx;
            feedback->current_vy = current_state_.vy;
            feedback->current_vz = current_state_.vz;
            feedback->current_altitude = current_state_.z;
            feedback->elapsed_time = elapsed;
        }
        goal_handle->publish_feedback(feedback);

        rate.sleep();
    }
    
    // 如果因为 rclcpp::ok() 退出，发送停止指令
    core_action::Velocity stop_vel(0, 0, 0, 0);
    auto twist = stop_vel.get_twist();
    twist.header.stamp = this->now();
    vel_pub_->publish(twist);
}

void VelocityActionServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    current_state_.x = msg->pose.pose.position.x;
    current_state_.y = msg->pose.pose.position.y;
    current_state_.z = msg->pose.pose.position.z;
    
    current_state_.vx = msg->twist.twist.linear.x;
    current_state_.vy = msg->twist.twist.linear.y;
    current_state_.vz = msg->twist.twist.linear.z;
    
    current_state_.valid = true;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocityActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
