#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "core_action/action/fly_by_velocity.hpp"
#include "core_action/velocity.hpp"

class VelocityActionClient : public rclcpp::Node {
public:
    using FlyByVelocity = core_action::action::FlyByVelocity;
    using GoalHandle = rclcpp_action::ClientGoalHandle<FlyByVelocity>;

    explicit VelocityActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("velocity_action_client", options) {
        // 创建 速度飞行Action 客户端
        this->client_ = rclcpp_action::create_client<FlyByVelocity>(this, "fly_by_velocity");
        RCLCPP_INFO(this->get_logger(), "Velocity Action 客户端 已初始化");
    }

    void send_goal(const core_action::Velocity& velocity,
                   float duration = 5.0f,
                   bool maintain_altitude = false, // 是否定高
                   float altitude_target = 0.0f // 目标高度
                ) {
        using namespace std::placeholders;

        // 等待 Action Server 可用
        if (!this->client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Velocity Action 服务端 超时未响应");
            return;
        }

        // 设置回调选项
        auto send_goal_options = rclcpp_action::Client<FlyByVelocity>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            std::bind(&VelocityActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&VelocityActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&VelocityActionClient::result_callback, this, _1);

        // 从 Velocity 对象构造 Goal
        auto goal_msg = FlyByVelocity::Goal();
        goal_msg.vx = velocity.get_vx();
        goal_msg.vy = velocity.get_vy();
        goal_msg.vz = velocity.get_vz();
        goal_msg.vyaw = velocity.get_vyaw();
        goal_msg.duration = duration;
        goal_msg.maintain_altitude = maintain_altitude;
        goal_msg.altitude_target = altitude_target;

        // 发送目标
        this->client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), 
            "已发送速度指令: vel=(%.2f, %.2f, %.2f) m/s, vyaw=%.2f rad/s, 持续=%.1fs", 
            velocity.get_vx(), velocity.get_vy(), velocity.get_vz(), 
            velocity.get_vyaw(), duration);
    }

    // 直接发送vel
    void send_goal(float vx, float vy, float vz, float vyaw = 0.0f,
                   float duration = 5.0f,
                   bool maintain_altitude = false,
                   float altitude_target = 0.0f)
    {
        core_action::Velocity velocity(vx, vy, vz, vyaw);
        send_goal(velocity, duration, maintain_altitude, altitude_target);
    }

    // 取消当前速度控制目标
    void cancel_goal() {
        if (goal_handle_) {
            RCLCPP_INFO(this->get_logger(), "取消当前速度控制");
            this->client_->async_cancel_goal(goal_handle_);
        } else {
            RCLCPP_WARN(this->get_logger(), "当前无活动速度控制任务可取消");
        }
    }

private:
    // 目标响应回调
    void goal_response_callback(const GoalHandle::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "速度控制目标被拒绝");
        } else {
            RCLCPP_INFO(this->get_logger(), "速度控制目标被接受");
            goal_handle_ = goal_handle;
        }
    }

    // 反馈回调
    void feedback_callback(
        GoalHandle::SharedPtr,
        const std::shared_ptr<const FlyByVelocity::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(),
            "反馈: 速度=(%.2f, %.2f, %.2f) m/s, 高度=%.2fm, 时间=%.1fs",
            feedback->current_vx, feedback->current_vy, feedback->current_vz,
            feedback->current_altitude,
            feedback->elapsed_time);
    }

    // 结果回调
    void result_callback(const GoalHandle::WrappedResult & result) {
        goal_handle_.reset();
        
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "速度控制被中止");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "速度控制被取消");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "未知的结果代码");
                return;
        }

        if (result.result->success) {
            RCLCPP_INFO(this->get_logger(),
                "速度控制完成！消息: %s, 时长: %.2fs, 高度: %.2fm",
                result.result->message.c_str(),
                result.result->actual_duration,
                result.result->final_altitude);
        } else {
            RCLCPP_WARN(this->get_logger(), "速度控制失败: %s", result.result->message.c_str());
        }
    }

    rclcpp_action::Client<FlyByVelocity>::SharedPtr client_;
    GoalHandle::SharedPtr goal_handle_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // 检查命令行参数
    if (argc < 4) {
        std::cout << "用法: velocity_action_client <vx> <vy> <vz> [vyaw] [duration] [maintain_altitude] [altitude_target]" << std::endl;
        std::cout << "示例: ros2 run core_action velocity_action_client 1.0 0.0 0.0 0.0 5.0" << std::endl;
        std::cout << "      前进 1m/s，持续 5 秒" << std::endl;
        std::cout << "示例: ros2 run core_action velocity_action_client 1.0 0.5 0.0 0.0 10.0 1 1.5" << std::endl;
        std::cout << "      前进+右移，保持 1.5m 高度，持续 10 秒" << std::endl;
        return 1;
    }

    auto client_node = std::make_shared<VelocityActionClient>();

    // 解析参数
    float vx = std::stof(argv[1]);
    float vy = std::stof(argv[2]);
    float vz = std::stof(argv[3]);
    float vyaw = (argc > 4) ? std::stof(argv[4]) : 0.0f;
    float duration = (argc > 5) ? std::stof(argv[5]) : 5.0f;
    bool maintain_altitude = (argc > 6) ? (std::stoi(argv[6]) != 0) : false;
    float altitude_target = (argc > 7) ? std::stof(argv[7]) : 0.0f;

    // 创建 Velocity 对象并发送
    core_action::Velocity velocity(vx, vy, vz, vyaw);
    RCLCPP_INFO(client_node->get_logger(), 
        "创建速度指令: (%.2f, %.2f, %.2f) m/s, vyaw=%.2f rad/s", 
        vx, vy, vz, vyaw);
    
    client_node->send_goal(velocity, duration, maintain_altitude, altitude_target);

    // 持续运行以接收反馈和结果
    rclcpp::spin(client_node);

    rclcpp::shutdown();
    return 0;
}
