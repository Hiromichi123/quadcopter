/**
 * @brief 简化版四旋翼控制节点 - 教学版本
 * 
 * 主要功能：
 * 1. 基本定点飞行（fly_to_target）
 * 2. 基本速度控制（fly_by_velocity）
 * 3. 基本的起飞、解锁和模式切换
 * 
 * @author Hiromichi123
 * @date 2026-01-17
 */

#include <memory>
#include <thread>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

// ROS2标准消息类型
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

// MAVROS消息和服务类型（用于与飞控通信）
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>

// 自定义消息类型
#include "ros2_tools/msg/lidar_pose.hpp"

// 简化的数据类型定义
#include "simple_types.hpp"

/**
 * @class QuadcopterMini
 * @brief 简化版四旋翼控制节点类
 * 
 * 飞行器的所有功能：
 * - 与MAVROS通信（发送位置/速度指令）
 * - 接收传感器数据（激光雷达位置）
 * - 实现基本飞行功能（定点飞行、速度控制）
 * 
 * 教学要点：
 * - ROS2节点通过继承rclcpp::Node类创建
 * - 使用Publisher发布消息，Subscription订阅消息，Client调用服务
 * - 飞行器控制需要循环发布指令，保持与飞控的通信（不低于2Hz）
 */
class QuadcopterMini : public rclcpp::Node {
public:
    float x, y, z;      // 当前位置
    float yaw;          // 当前航向角

    /**
     * @brief 构造函数 - 初始化节点
     * 
     * 构造函数完成：
     * 1. 初始化成员变量
     * 2. 创建订阅者（subscriber）接收数据
     * 3. 创建发布者（publisher）发送指令
     * 4. 创建服务客户端（client）调用飞控服务
     */
    QuadcopterMini() : Node("quadcopter_mini_node") {
        // 初始化位置变量，如果不初始化，默认为0.0
        x = y = z = yaw = 0.0;
        
        // 创建控制频率（建议20Hz/50Hz）
        rate = std::make_shared<rclcpp::Rate>(20.0);
        
        // 话题订阅组
        // 激光雷达位置数据订阅（获取飞行器当前位置）
        lidar_sub = this->create_subscription<ros2_tools::msg::LidarPose>(
            "lidar_data",  // 话题名称
            10,            // 队列长度
            std::bind(&QuadcopterMini::lidar_callback, this, std::placeholders::_1)  // 回调函数
        );
        
        // 飞控MAVROS状态订阅（获取飞行器解锁状态和飞行模式）
        state_sub = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 
            10, 
            std::bind(&QuadcopterMini::state_callback, this, std::placeholders::_1)
        );
        
        // 话题发布组
        // 位置发布器（用于定点飞行）
        pos_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", 
            10
        );
        
        // 速度发布器（用于速度控制飞行）
        vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 
            10
        );
        
        // 服务客户端组
        // 解锁/上锁服务
        arming_client = this->create_client<mavros_msgs::srv::CommandBool>(
            "mavros/cmd/arming"
        );
        
        // 设置飞行模式服务
        set_mode_client = this->create_client<mavros_msgs::srv::SetMode>(
            "mavros/set_mode"
        );
        
        // 初始化飞控状态
        current_state = std::make_shared<mavros_msgs::msg::State>();
        
        RCLCPP_INFO(this->get_logger(), "四旋翼控制节点初始化完成");
    }
    
    /**
     * @brief 启动自旋线程处理回调
     * 
     * ROS2需要持续处理订阅的消息，这个函数创建一个独立线程
     * 在后台不断调用spin_some()来处理接收到的消息
     * 
     * 教学要点：
     * - spin_some()会处理所有待处理的回调函数
     * - 使用独立线程可以让主线程专注于飞行控制逻辑
     */
    void start_spin_thread() {
        spin_thread = std::make_shared<std::thread>([this]() {
            while (rclcpp::ok()) {
                // 处理所有待处理的回调
                rclcpp::spin_some(shared_from_this());
                // 短暂休眠，避免CPU占用过高
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });
        
        // 注册关闭时的清理函数
        rclcpp::on_shutdown([this]() {
            if (spin_thread && spin_thread->joinable()) {
                spin_thread->join();  // 等待线程结束
            }
        });
        
        RCLCPP_INFO(this->get_logger(), "消息处理线程已启动");
    }
    
    /**
     * @brief 起飞前检查和准备，一套八股流程
     * 
     * 这个函数负责：
     * 1. 预发布起飞点位置（MAVROS要求先发布位置再切换模式）
     * 2. 等待与飞控建立连接
     * 3. 解锁飞行器
     * 4. 切换到OFFBOARD模式
     * 5. 飞到起飞点
     * 
     * 教学要点：
     * - OFFBOARD模式：由外部计算机控制飞行器
     * - 必须先发布位置指令，再切换OFFBOARD模式
     * - 解锁（arm）：允许电机启动
     */
    void prepare_for_flight() {
        RCLCPP_INFO(this->get_logger(), "=== 开始起飞前准备 ===");
        
        // 创建起飞点目标
        Target takeoff_point(0.0, 0.0, 0.5, 0.0);
        
        // 步骤1：预发布起飞点位置
        // MAVROS要求在切换到OFFBOARD模式前，必须先发布一些位置预指令
        RCLCPP_INFO(this->get_logger(), "预发布...");
        for (int i = 0; i < 100; i++) {
            auto pose = takeoff_point.to_pose_stamped();
            pos_pub->publish(pose);
            rate->sleep();
        }
        
        // 步骤2：循环检查并设置飞行器状态
        rclcpp::Time last_request = this->now();
        
        while (rclcpp::ok()) {
            // 持续发布位置指令
            auto pose = takeoff_point.to_pose_stamped();
            pos_pub->publish(pose);
            
            // 检查是否需要解锁
            if (!current_state->armed && 
                (this->now() - last_request > rclcpp::Duration::from_seconds(1.0))) {
                
                // 调用解锁服务
                auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                request->value = true;
                
                arming_client->async_send_request(request);
                RCLCPP_INFO(this->get_logger(), "正在尝试解锁...");
                last_request = this->now();
            }
            // 检查是否需要切换模式
            else if (current_state->mode != "OFFBOARD" && 
                     (this->now() - last_request > rclcpp::Duration::from_seconds(1.0))) {
                
                // 调用设置模式服务
                auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                request->custom_mode = "OFFBOARD";
                
                set_mode_client->async_send_request(request);
                RCLCPP_INFO(this->get_logger(), "正在切换到OFFBOARD模式...");
                last_request = this->now();
            }
            // 检查是否已经解锁且进入OFFBOARD模式
            else if (current_state->armed && current_state->mode == "OFFBOARD") {
                RCLCPP_INFO(this->get_logger(), "解锁成功，OFFBOARD模式已激活");
                
                // 飞到起飞点
                fly_to_target(takeoff_point);
                RCLCPP_INFO(this->get_logger(), "已到达起飞点");
                break;
            }
            
            rate->sleep();
        }
    }
    
    /**
     * @brief 定点飞行功能
     * @param target 目标点（包含xyz坐标和yaw角）
     * @param timeout_sec 超时时间，默认10s
     * @param stable_time_sec 稳定时间，默认0.25s
     * @param check_distance 到达判定距离，默认0.1m
     * 
     * 原理：
     * 1. 循环发布目标位置
     * 2. 检查当前位置与目标的距离
     * 3. 当距离小于check_distance且持续stable_time_sec时间，认为到达
     * 4. 如果超过timeout_sec仍未到达，则超时退出
     * 
     * 教学要点：
     * - 飞控内部有位置控制器，会自动规划路径飞到目标点（轮椅法）
     * - 新手只需要持续发布目标位置即可
     * - 但要注意是否到达，避免无限等待。中途有没有异常和其他挂起（进阶操作）
     */
    void fly_to_target(const Target& target, 
                       float timeout_sec = 10.0f, 
                       float stable_time_sec = 0.25f,
                       float check_distance = 0.1f) {
        
        RCLCPP_INFO(this->get_logger(), 
            "飞往目标点: (%.2f, %.2f, %.2f, yaw=%.2f)", 
            target.x, target.y, target.z, target.yaw);
        
        // 记录开始时间
        auto start_time = this->now();
        // 稳定计数器：记录连续多少次检查都在目标范围内
        int stable_count = 0;
        // 需要的稳定帧数 = 稳定时间 × 控制频率
        const int required_stable_frames = static_cast<int>(stable_time_sec * 20);
        
        // 主控制循环
        while (rclcpp::ok()) {
            // 检查1：是否超时
            if ((this->now() - start_time).seconds() > timeout_sec) {
                RCLCPP_WARN(this->get_logger(), "⚠ 飞行超时！");
                break;
            }
            
            // 发布目标位置指令
            auto pose = target.to_pose_stamped();
            pos_pub->publish(pose);
            
            // 检查2：是否到达目标点
            if (check_position(target, check_distance)) {
                // 持续稳定时间，认为到达
                if (++stable_count >= required_stable_frames) {
                    RCLCPP_INFO(this->get_logger(), "已到达目标点");
                    break;
                }
            } else {
                stable_count = 0; // 脱离目标范围，重置计数
            }
            
            rate->sleep(); // 按照控制频率休眠
        }
    }
    
    /**
     * @brief 速度控制飞行
     * @param velocity 速度指令（包含vx, vy, vz, vyaw）
     * 
     * 教学要点：
     * - 速度控制是直接控制飞行器的v
     * - 与位置控制不同，速度控制需要持续发布，否则飞行器会停止
     * - 适合需要精确控制运动过程的场景（如巡线、跟踪等）
     */
    void fly_by_velocity(const Velocity& velocity) {
        auto twist = velocity.to_twist_stamped(); // 转换为ROS消息并发布
        vel_pub->publish(twist);
    }
    
    /**
     * @brief 主任务循环
     */
    void run_mission() {
        RCLCPP_INFO(this->get_logger(), "开始执行定点任务...");
        
        // 第一个目标点：向前飞1米
        Target point1(1.0, 0.0, 0.5, 0.0);
        fly_to_target(point1);
        RCLCPP_INFO(this->get_logger(), "完成目标点 1");
        
        // 第二个目标点：向右飞1米
        Target point2(1.0, 1.0, 0.5, 0.0);
        fly_to_target(point2);
        RCLCPP_INFO(this->get_logger(), "完成目标点 2");
        
        // 第三个目标点：返回原点并爬升转向
        Target point3(0.0, 0.0, 1.0, 1.57); // yaw约90度
        fly_to_target(point3);
        RCLCPP_INFO(this->get_logger(), "完成目标点 3");
        
        // 使用速度控制的示例
        /*
        RCLCPP_INFO(this->get_logger(), "开始速度控制飞行");
        Velocity vel_forward(0.1, 0.0, 0.0, 0.0);  // 向前0.1m/s
        
        auto start_time = this->now();
        while (rclcpp::ok() && (this->now() - start_time).seconds() < 5.0) {
            fly_by_velocity(vel_forward);
            rate->sleep();
        }
        
        // 停止
        Velocity vel_stop(0.0, 0.0, 0.0, 0.0);
        fly_by_velocity(vel_stop);
        RCLCPP_INFO(this->get_logger(), "速度控制飞行结束");
        */
        
        // 保持悬停
        Target hover(x, y, z, yaw);  // 悬停在当前位置
        while (rclcpp::ok()) {
            auto pose = hover.to_pose_stamped();
            pos_pub->publish(pose);
            rate->sleep();
        }
    }

    
private:
    /**
     * @brief 检查是否到达目标位置
     * @param target 目标点
     * @param distance 允许的误差距离m
     * @return true 如果在误差范围内
     */
    bool check_position(const Target& target, float distance=0.1f, float yaw_tolerance=0.1f) {
        // 计算与目标点的欧氏距离
        float dist_sq = std::pow(x - target.x, 2) + 
                       std::pow(y - target.y, 2) + 
                       std::pow(z - target.z, 2);
        float dist = std::sqrt(dist_sq);
        
        // 计算航向角误差
        float yaw_error = std::abs(yaw - target.yaw);
        
        // 每2秒打印一次距离信息（避免刷屏）
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "当前距离目标: %.2fm, 航向角误差: %.2frad", dist, yaw_error);
        
        // 判断是否到达
        return (dist < distance && yaw_error < yaw_tolerance);
    }
    
    /**
     * @brief 激光雷达位置回调函数
     * 
     * 当接收到新的激光雷达位置数据时，这个函数会被自动调用
     * 更新飞行器的当前位置信息
     */
    void lidar_callback(const ros2_tools::msg::LidarPose::SharedPtr msg) {
        x = msg->x;
        y = msg->y;
        z = msg->z;
        yaw = msg->yaw;
    }
    
    /**
     * @brief MAVROS状态回调函数
     * 
     * 当接收到新的飞控状态时，这个函数会被自动调用
     * 保存当前的解锁状态和飞行模式
     */
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        current_state = msg;
    }
    
    // ROS2订阅者
    rclcpp::Subscription<ros2_tools::msg::LidarPose>::SharedPtr lidar_sub;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    
    // ROS2发布者
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub;
    
    // ROS2服务客户端
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    
    std::shared_ptr<mavros_msgs::msg::State> current_state; // 飞控状态
    std::shared_ptr<rclcpp::Rate> rate; // 控制频率
    std::shared_ptr<std::thread> spin_thread; // 自旋线程
};

/**
 * @brief 主函数入口
 * 
 * 程序执行流程：
 * 1. 初始化ROS2
 * 2. 创建四旋翼节点
 * 3. 启动消息处理线程
 * 4. 执行起飞前检查
 * 5. 执行任务
 * 6. 关闭ROS2
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv); // 初始化ROS2
    
    auto quad_node = std::make_shared<QuadcopterMini>(); // 创建四旋翼节点
    
    quad_node->start_spin_thread(); // 启动消息处理线程
    
    quad_node->prepare_for_flight(); // 起飞前检查和准备
    
    quad_node->run_mission(); // 执行任务
    
    rclcpp::shutdown(); // 关闭ROS2
    
    return 0;
}
