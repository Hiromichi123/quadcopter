#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "quadcopter.h"
#include "target.hpp"
#include "velocity.hpp"
#include "path.hpp"

#define pi 3.14
constexpr float Default_Pos_Check_Distance=0.25; 

class quadcopter;
class Target;
class Velocity;
class Path;

class flight_controller : public rclcpp::Node {
public:
    flight_controller(std::shared_ptr<quadcopter> quad_node);
    void fly_to_target(const Target& target, float timeout_sec = 20.0f, float stable_time_sec = 0.25f, int frame_rate = 20); // target定点移动
    void fly_to_target_pid(const Target& target, float timeout_sec = 20.0f, float stable_time_sec = 0.25f, int frame_rate = 20); // target定点移动 (PID控制)
    void fly_by_velocity(const Velocity& velocity); // velocity速度飞行，单次发布
    void fly_by_vel_duration(const Velocity& velocity, float duration); // velocity速度飞行，发布持续duration
    void fly_by_path(Path* path); // 路径航点飞行，已兼容target版本

private:
    std::weak_ptr<quadcopter> quad_node; // 一个不参与生命周期的观察句柄
    std::shared_ptr<rclcpp::Rate> rate;

    // 自身位置检查，distance为误差默认0.1
    bool pos_check(const Target& target, float distance = Default_Pos_Check_Distance);
    // 重载严格检查，多维误差
    bool pos_check(const Target& target, float distance_x, float distance_y, float distance_z);
};
#endif
