#include <memory>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "core_action/action/navigate_to_target.hpp"
#include "core_action/action/fly_by_velocity.hpp"
#include "core_action/target.hpp"
#include "core_action/velocity.hpp"
#include "core_action/path.hpp"

using namespace std::chrono_literals;

// 综合飞行任务示例
class MixedFlightMission : public rclcpp::Node {
public:
    using NavigateToTarget = core_action::action::NavigateToTarget;
    using FlyByVelocity = core_action::action::FlyByVelocity;
    using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToTarget>;
    using VelGoalHandle = rclcpp_action::ClientGoalHandle<FlyByVelocity>;

    MixedFlightMission() : Node("mixed_flight_mission")
    {
        // 定点client
        nav_client_ = rclcpp_action::create_client<NavigateToTarget>(this, "navigate_to_target");
        
        // 速度控制client
        vel_client_ = rclcpp_action::create_client<FlyByVelocity>(this, "fly_by_velocity");
        
        RCLCPP_INFO(this->get_logger(), "双任务客户端已初始化");
    }

    // 任务示例：起飞 -> 定点飞行 -> 速度巡航 -> 定点返回 -> 降落
    void execute_mission() {
        RCLCPP_INFO(this->get_logger(), "开始执行混合任务");
        
        RCLCPP_INFO(this->get_logger(), "[1/5] 起飞到 1.5m 高度");
        navigate_to_sync(core_action::Target(0.0, 0.0, 1.5, 0.0));
        std::this_thread::sleep_for(2s);
        
        RCLCPP_INFO(this->get_logger(), "[2/5] 定点飞行到前方 2m");
        navigate_to_sync(core_action::Target(2.0, 0.0, 1.5, 0.0));
        std::this_thread::sleep_for(1s);
        
        RCLCPP_INFO(this->get_logger(), "[3/5] 速度控制：向右移动 5 秒");
        core_action::Velocity right_vel(0.0, 0.5, 0.0, 0.0);  // 0.5m/s向右
        fly_velocity_sync(right_vel, 5.0, true, 1.5);  // 1.5m定高
        std::this_thread::sleep_for(1s);
        
        RCLCPP_INFO(this->get_logger(), "[4/5] 速度控制：前进并旋转 3 秒");
        core_action::Velocity forward_rotate(0.5, 0.0, 0.0, 0.3);  // 前进 + 旋转
        fly_velocity_sync(forward_rotate, 3.0, true, 1.5);
        std::this_thread::sleep_for(1s);
        
        RCLCPP_INFO(this->get_logger(), "[5/5] 返回原点");
        navigate_to_sync(core_action::Target(0.0, 0.0, 1.5, 0.0));
        std::this_thread::sleep_for(1s);
        
        RCLCPP_INFO(this->get_logger(), "降落");
        navigate_to_sync(core_action::Target(0.0, 0.0, 0.05, 0.0));
        
        RCLCPP_INFO(this->get_logger(), "任务已完成");
    }

    /**
     * @brief 演示速度控制的各种用法
     */
    void velocity_control_demo() {
        
        // 1. 基本速度控制：前进
        RCLCPP_INFO(this->get_logger(), "定速1m/s，持续3s");
        fly_velocity_sync(core_action::Velocity(1.0, 0.0, 0.0), 3.0);
        std::this_thread::sleep_for(1s);
        
        // 2. 横向移动（保持高度）
        RCLCPP_INFO(this->get_logger(), "向右0.5定数巡航，1.5m定高，持续4s");
        fly_velocity_sync(core_action::Velocity(0.0, 0.5, 0.0), 4.0, true, 1.5);
        std::this_thread::sleep_for(1s);
        
        // 3. 复杂运动：前进+右移+上升+旋转
        RCLCPP_INFO(this->get_logger(), "复合运动：前进+右移+旋转，持续5s");
        fly_velocity_sync(core_action::Velocity(0.5, 0.3, 0.0, 0.2), 5.0, true, 2.0);
        std::this_thread::sleep_for(1s);
        
        // 4. 圆周运动（通过连续的速度指令）
        RCLCPP_INFO(this->get_logger(), "圆周运动模拟");
        for (int i = 0; i < 8; i++) {
            float angle = i * M_PI / 4.0;  // 每次 45 度
            float vx = 0.5 * std::cos(angle);
            float vy = 0.5 * std::sin(angle);
            fly_velocity_sync(core_action::Velocity(vx, vy, 0.0), 1.0, true, 1.5);
        }
    }

private:
    // 定点飞行同步执行
    bool navigate_to_sync(const core_action::Target& target) {
        if (!nav_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "导航 Action 服务器不可用");
            return false;
        }

        auto goal = NavigateToTarget::Goal();
        goal.x = target.get_x();
        goal.y = target.get_y();
        goal.z = target.get_z();
        goal.yaw = target.get_yaw();
        goal.timeout_sec = 30.0;
        goal.stable_time_sec = 2.0;
        goal.tolerance = 0.15;

        auto goal_handle_future = nav_client_->async_send_goal(goal);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) 
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "发送导航目标失败");
            return false;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "导航目标被拒绝");
            return false;
        }

        auto result_future = nav_client_->async_get_result(goal_handle);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) 
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "获取导航结果失败");
            return false;
        }

        return result_future.get().result->success;
    }

    // 速度控制同步执行
    bool fly_velocity_sync(const core_action::Velocity& velocity, 
                          float duration,
                          bool maintain_altitude = false,
                          float altitude_target = 0.0f) 
    {
        if (!vel_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "速度 Action 服务器不可用");
            return false;
        }

        auto goal = FlyByVelocity::Goal();
        goal.vx = velocity.get_vx();
        goal.vy = velocity.get_vy();
        goal.vz = velocity.get_vz();
        goal.vyaw = velocity.get_vyaw();
        goal.duration = duration;
        goal.maintain_altitude = maintain_altitude;
        goal.altitude_target = altitude_target;

        auto goal_handle_future = vel_client_->async_send_goal(goal);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) 
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "发送速度指令失败");
            return false;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "速度指令被拒绝");
            return false;
        }

        auto result_future = vel_client_->async_get_result(goal_handle);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) 
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "获取速度控制结果失败");
            return false;
        }

        return result_future.get().result->success;
    }

    rclcpp_action::Client<NavigateToTarget>::SharedPtr nav_client_;
    rclcpp_action::Client<FlyByVelocity>::SharedPtr vel_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MixedFlightMission>();

    // 选择要执行的任务
    if (argc > 1 && std::string(argv[1]) == "velocity_demo") {
        node->velocity_control_demo();
    } else {
        node->execute_mission();
    }

    rclcpp::shutdown();
    return 0;
}
