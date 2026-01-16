#ifndef SIMPLE_TYPES_HPP
#define SIMPLE_TYPES_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

/**
 * @brief 目标点类
 * 
 * Target类用于表示飞行器需要到达的目标位置和姿态
 * 包含了三维位置坐标(x,y,z)和航向角(yaw)
 * 
 * 教学要点：
 * - 将位置信息封装成ROS2的PoseStamped消息格式
 * - PoseStamped包含位置(position)和四元数姿态(orientation)
 * - 使用归一化yaw角来控制飞行器的朝向
 */
class Target {
public:
    // 位置信息（公开成员，方便直接访问）
    float x;      // X轴位置（米）
    float y;      // Y轴位置（米）
    float z;      // Z轴位置（米，高度）
    float yaw;    // 航向角（弧度）
    
    // 标记是否已到达目标点（可有可无）
    bool reached;
    
    /**
     * @brief 默认构造函数
     */
    Target() : x(0), y(0), z(0), yaw(0), reached(false) {}
    
    /**
     * @brief 带参数的构造函数
     * @param x_ X轴坐标
     * @param y_ Y轴坐标
     * @param z_ Z轴坐标（高度）
     * @param yaw_ 航向角（弧度）
     */
    Target(float x_, float y_, float z_, float yaw_) 
        : x(x_), y(y_), z(z_), yaw(yaw_), reached(false) {}
    
    /**
     * @brief 转换为ROS2的PoseStamped消息
     * 
     * 教学要点：
     * - PoseStamped是ROS2中表示带时间戳的位置姿态的标准消息
     * - 位置用xyz直接表示
     * - 姿态用四元数表示，这里我们只关心yaw角（绕Z轴旋转）
     * - 四元数转换公式：yaw -> (0, 0, sin(yaw/2), cos(yaw/2))
     * 
     * @return geometry_msgs::msg::PoseStamped ROS2位置姿态消息
     */
    geometry_msgs::msg::PoseStamped to_pose_stamped() const {
        geometry_msgs::msg::PoseStamped pose;
        
        pose.header.stamp = rclcpp::Clock().now(); // 设置时间戳为当前时间
        pose.header.frame_id = "map"; // 设置坐标系为map（全局坐标系）
        
        // 设置位置
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        
        // 将yaw角转换为四元数
        // 四元数是一种表示三维旋转的数学工具
        // 对于只有yaw角的情况，四元数的x和y分量为0
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);
        
        return pose;
    }
};

/**
 * @brief 速度类
 * 
 * Velocity类用于表示飞行器的运动速度
 * 包含三维线速度(vx,vy,vz)和角速度(vyaw)
 * 
 * 教学要点：
 * - 线速度控制飞行器在xyz三个轴上的移动速度
 * - 角速度控制飞行器的旋转速度，线速度为m/s，角速度为rad/s
 */
class Velocity {
public:
    // 速度信息（公开成员，方便直接访问）
    float vx;     // X轴速度（米/秒）
    float vy;     // Y轴速度（米/秒）
    float vz;     // Z轴速度（米/秒，上升/下降速度）
    float vyaw;   // 航向角速度（弧度/秒）
    
    /**
     * @brief 默认构造函数
     */
    Velocity() : vx(0), vy(0), vz(0), vyaw(0) {}
    
    /**
     * @brief 带参数的构造函数
     * @param vx_ X轴速度
     * @param vy_ Y轴速度
     * @param vz_ Z轴速度
     * @param vyaw_ 航向角速度（默认为0）
     */
    Velocity(float vx_, float vy_, float vz_, float vyaw_ = 0.0) 
        : vx(vx_), vy(vy_), vz(vz_), vyaw(vyaw_) {}
    
    /**
     * @brief 转换为ROS2的TwistStamped消息
     * 
     * 教学要点：
     * - TwistStamped是ROS2中表示带时间戳的速度的标准消息
     * - linear表示线速度（xyz方向的移动速度）
     * - angular表示角速度（绕xyz轴的旋转速度）
     * 
     * @return geometry_msgs::msg::TwistStamped ROS2速度消息
     */
    geometry_msgs::msg::TwistStamped to_twist_stamped() const {
        geometry_msgs::msg::TwistStamped twist;
        
        // 设置时间戳
        twist.header.stamp = rclcpp::Clock().now();
        
        // 设置线速度
        twist.twist.linear.x = vx;
        twist.twist.linear.y = vy;
        twist.twist.linear.z = vz;
        
        // 设置角速度（只设置yaw轴）
        twist.twist.angular.x = 0.0;  // roll角速度为0
        twist.twist.angular.y = 0.0;  // pitch角速度为0
        twist.twist.angular.z = vyaw; // yaw角速度
        
        return twist;
    }
};

#endif // SIMPLE_TYPES_HPP
