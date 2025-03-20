#ifndef QUAD_NODE_H
#define QUAD_NODE_H

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>

#include "ros2_tools/msg/lidar_pose.hpp"
#include "vision/msg/vision.hpp"

class flight_controller;

class quadcopter : public rclcpp::Node {
public:
    float x, y, z, yaw; // 全局位置
    float vx, vy, vz; // 速度
    float ax, ay, az; // 加速度
    float roll, pitch; // 姿态角
    float dr, dp; // 角速度

    friend class flight_controller; // 为了保险

    quadcopter();
    void quad_init(); // 初始化quad节点控制流程
    void flight_ctrl_init(); //初始化飞行控制类
    void start_spin_thread(); // 注册shutdown回调并创建spin线程
    void pre_flight_checks_loop(); // 起飞前检查
    void main_loop(); // 主循环

    void arm_and_takeoff(float altitude); // 另一种起飞

private:
    std::shared_ptr<flight_controller> flight_ctrl; // 持有飞行控制类，增加引用次数

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<ros2_tools::msg::LidarPose>::SharedPtr lidar_sub;
    rclcpp::Subscription<vision::msg::Vision>::SharedPtr vision_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub;

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    
    std::shared_ptr<std::thread> spin_thread;
    std::shared_ptr<rclcpp::Rate> rate;

    // lidar数据回调
    std::shared_ptr<ros2_tools::msg::LidarPose> lidar_pos;
    void lidar_pose_cb(const ros2_tools::msg::LidarPose::SharedPtr msg);

    // mavros状态回调
    std::shared_ptr<mavros_msgs::msg::State> current_state;
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg);

    // vision数据回调
    std::shared_ptr<vision::msg::Vision> vision_msg;
    void vision_sub_cb(const vision::msg::Vision::SharedPtr msg);
};
#endif
