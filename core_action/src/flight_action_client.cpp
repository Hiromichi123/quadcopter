#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "core_action/action/navigate_to_target.hpp"
#include "core_action/target.hpp"

class FlightActionClient : public rclcpp::Node {
public:
    using NavigateToTarget = core_action::action::NavigateToTarget;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToTarget>;

    explicit FlightActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("flight_action_client", options)
    {
        this->client_ = rclcpp_action::create_client<NavigateToTarget>(this, "navigate_to_target");
        
        RCLCPP_INFO(this->get_logger(), "Flight Action 客户端 已初始化");
    }

    // 使用 Target 对象发送目标（推荐）
    void send_goal(const core_action::Target& target,
                   float timeout_sec = 10.0f,
                   float stable_time_sec = 0.5f,
                   float tolerance = 0.1f)
    {
        using namespace std::placeholders;

        // 等待 Action Server 可用
        if (!this->client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action 服务端 超时未响应");
            return;
        }

        // 设置回调选项
        auto send_goal_options = rclcpp_action::Client<NavigateToTarget>::SendGoalOptions();
        
        // 绑定目标响应、反馈、结果回调
        send_goal_options.goal_response_callback =
            std::bind(&FlightActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&FlightActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&FlightActionClient::result_callback, this, _1);

        // 从 Target 构造目标消息
        auto goal_msg = NavigateToTarget::Goal();
        goal_msg.x = target.get_x();
        goal_msg.y = target.get_y();
        goal_msg.z = target.get_z();
        goal_msg.yaw = target.get_yaw();
        goal_msg.timeout_sec = timeout_sec;
        goal_msg.stable_time_sec = stable_time_sec;
        goal_msg.tolerance = tolerance;

        // 发送目标
        this->client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "已发送目标: (%.2f, %.2f, %.2f, %.2f rad)", 
                    target.get_x(), target.get_y(), target.get_z(), target.get_yaw());
    }

    // 便捷方法：直接使用坐标发送目标
    void send_goal(float x, float y, float z, float yaw,
                   float timeout_sec = 10.0f,
                   float stable_time_sec = 0.5f,
                   float tolerance = 0.1f)
    {
        core_action::Target target(x, y, z, yaw);
        send_goal(target, timeout_sec, stable_time_sec, tolerance);
    }

    void cancel_goal()
    {
        if (goal_handle_) {
            RCLCPP_INFO(this->get_logger(), "取消当前目标");
            this->client_->async_cancel_goal(goal_handle_);
        } else {
            RCLCPP_WARN(this->get_logger(), "当前无活动目标可取消");
        }
    }

private:
    // 目标响应回调
    void goal_response_callback(const GoalHandle::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "目标被拒绝");
        } else {
            RCLCPP_INFO(this->get_logger(), "目标被接受，等待结果");
            goal_handle_ = goal_handle;
        }
    }

    // 反馈回调
    void feedback_callback(
        GoalHandle::SharedPtr,
        const std::shared_ptr<const NavigateToTarget::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(),
            "反馈: 距离=%.2fm, 位置=(%.2f,%.2f,%.2f), 偏航=%.2f, 时间=%.1fs, 稳定=%d",
            feedback->distance_remaining,
            feedback->current_x, feedback->current_y, feedback->current_z,
            feedback->current_yaw,
            feedback->elapsed_time,
            feedback->stable_count);
    }

    // 结果回调
    void result_callback(const GoalHandle::WrappedResult & result)
    {
        goal_handle_.reset();
        
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "goal被中止");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "goal被取消");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "未知的结果");
                return;
        }

        if (result.result->success) {
            RCLCPP_INFO(this->get_logger(),
                "已到达目标点，消息: %s, 最终距离: %.3fm, 总时间: %.2fs",
                result.result->message.c_str(),
                result.result->final_distance,
                result.result->total_time);
        } else {
            RCLCPP_WARN(this->get_logger(),
                "未能到达目标点，消息: %s, 最终距离: %.3fm",
                result.result->message.c_str(),
                result.result->final_distance);
        }
    }

    rclcpp_action::Client<NavigateToTarget>::SharedPtr client_;
    GoalHandle::SharedPtr goal_handle_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // 检查命令行参数
    if (argc < 5) {
        std::cout << "如何使用: flight_action_client <x> <y> <z> <yaw> [timeout] [stable_time] [tolerance]" << std::endl;
        std::cout << "示例: ros2 run core_action flight_action_client 1.0 2.0 1.5 0.0" << std::endl;
        return 1;
    }

    // 解析参数
    float x = std::stof(argv[1]);
    float y = std::stof(argv[2]);
    float z = std::stof(argv[3]);
    float yaw = std::stof(argv[4]);
    float timeout_sec = (argc > 5) ? std::stof(argv[5]) : 10.0f;
    float stable_time_sec = (argc > 6) ? std::stof(argv[6]) : 0.5f;
    float tolerance = (argc > 7) ? std::stof(argv[7]) : 0.1f;

    // 创建客户端节点
    auto client_node = std::make_shared<FlightActionClient>();

    // 创建 Target 对象并发送目标
    core_action::Target target(x, y, z, yaw);
    RCLCPP_INFO(client_node->get_logger(), "创建目标: (%.2f, %.2f, %.2f, %.2f rad)", 
                x, y, z, yaw);
    
    client_node->send_goal(target, timeout_sec, stable_time_sec, tolerance);

    // 持续运行以接收反馈和结果
    rclcpp::spin(client_node);

    rclcpp::shutdown();
    return 0;
}
