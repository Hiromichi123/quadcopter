#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "core_action/action/navigate_to_target.hpp"
#include "core_action/target.hpp"
#include "core_action/path.hpp"

// Target 和 Path 类来简化多航点路径任务
class AdvancedFlightClient : public rclcpp::Node {
public:
    using NavigateToTarget = core_action::action::NavigateToTarget;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToTarget>;

    explicit AdvancedFlightClient() : Node("advanced_flight_client") {
        client_ = rclcpp_action::create_client<NavigateToTarget>(this, "navigate_to_target");
        RCLCPP_INFO(this->get_logger(), "路径飞行客户端已初始化");
    }

    // 执行单个目标任务
    void fly_to_single_target() {
        core_action::Target target1(1.0, 2.0, 1.5, 1.57);
        send_goal(target1);
    }

    // 执行多航点路径任务
    void fly_path_mission() {
        core_action::Path mission_path; // 创建路径任务
        
        // 添加航点
        mission_path.add_waypoint(0.0, 0.0, 1.5, 0.0);    // 起飞
        mission_path.add_waypoint(2.0, 0.0, 1.5, 0.0);    // 前进
        mission_path.add_waypoint(2.0, 2.0, 1.5, 1.57);   // 右转
        mission_path.add_waypoint(0.0, 2.0, 1.5, 3.14);   // 左转
        mission_path.add_waypoint(0.0, 0.0, 1.5, 0.0);    // 返回
        mission_path.add_waypoint(0.0, 0.0, 0.3, 0.0);    // 降落
        
        RCLCPP_INFO(this->get_logger(), "路径包含 %zu 个航点", mission_path.size());
        
        // 依次执行所有航点
        core_action::Target waypoint;
        while (mission_path.get_next_waypoint(waypoint)) {
            RCLCPP_INFO(this->get_logger(), 
                "前往航点: (%.2f, %.2f, %.2f, %.2f)", 
                waypoint.get_x(), waypoint.get_y(), 
                waypoint.get_z(), waypoint.get_yaw());
            
            send_goal_sync(waypoint);
            
            if (waypoint.reached) {
                RCLCPP_INFO(this->get_logger(), "航点已到达");
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "路径任务全部完成！");
    }

    // 发送目标（异步非阻塞）
    void send_goal(const core_action::Target& target) {
        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action 服务器不可用");
            return;
        }

        auto goal_msg = NavigateToTarget::Goal();
        goal_msg.x = target.get_x();
        goal_msg.y = target.get_y();
        goal_msg.z = target.get_z();
        goal_msg.yaw = target.get_yaw();
        goal_msg.timeout_sec = 10.0f;
        goal_msg.stable_time_sec = 0.5f;
        goal_msg.tolerance = 0.1f;

        auto send_goal_options = rclcpp_action::Client<NavigateToTarget>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandle::WrappedResult& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "目标达成！");
            }
        };

        client_->async_send_goal(goal_msg, send_goal_options);
    }

    // 发送目标（同步阻塞）
    void send_goal_sync(core_action::Target& target) {
        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action 服务器不可用");
            return;
        }

        auto goal_msg = NavigateToTarget::Goal();
        goal_msg.x = target.get_x();
        goal_msg.y = target.get_y();
        goal_msg.z = target.get_z();
        goal_msg.yaw = target.get_yaw();
        goal_msg.timeout_sec = 10.0f;
        goal_msg.stable_time_sec = 0.5f;
        goal_msg.tolerance = 0.1f;

        auto goal_handle_future = client_->async_send_goal(goal_msg);
        
        // 等待服务器接受
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "发送目标失败");
            return;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "目标被拒绝");
            return;
        }

        // 等待结果
        auto result_future = client_->async_get_result(goal_handle);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "获取结果失败");
            return;
        }

        auto result = result_future.get();
        if (result.result->success) {
            target.reached = true;
            RCLCPP_INFO(this->get_logger(), "目标成功到达！");
        } else {
            RCLCPP_WARN(this->get_logger(), "目标未到达: %s", result.result->message.c_str());
        }
    }

private:
    rclcpp_action::Client<NavigateToTarget>::SharedPtr client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AdvancedFlightClient>();

    // 示例1: 单个目标
    // node->fly_to_single_target();

    // 示例2: 多航点路径
    node->fly_path_mission();

    rclcpp::shutdown();
    return 0;
}
