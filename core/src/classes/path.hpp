#ifndef PATH_HPP
#define PATH_HPP

#include <vector>
#include <iostream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "target.hpp"

class Path {
public:
    size_t current_index = 0;
    std::vector<Target> waypoints;

    // 注册航点（PoseStamped 版） — 转换为 Target 存储
    void add_waypoint(const geometry_msgs::msg::PoseStamped& waypoint) {
        waypoints.emplace_back(waypoint);
    }

    // 注册航点 (Target 版)
    void add_waypoint(const Target& waypoint) {
        waypoints.push_back(waypoint);
    }

    // 注册航点 (直接参数版)
    void add_waypoint(float x, float y, float z, float yaw = 0.0f) {
        waypoints.emplace_back(x, y, z, yaw);
    }

    // 删除航点
    void remove_waypoint(size_t erase_num) {
        if (waypoints.empty()) {
            std::cerr << "Path:路径已经删空！" << std::endl;
            return;
        }

        if (erase_num < waypoints.size()) {
            waypoints.erase(waypoints.begin() + erase_num);
        } else {
            std::cerr << "Path:非法航点号:" << erase_num << std::endl;
        }
    }

    // 获取下一个航点（以 Target 形式返回）
    bool get_next_waypoint(Target& waypoint) {
        if (current_index < waypoints.size()) {
            waypoint = waypoints[current_index++];
            return true;
        } else {
            current_index = 0;
            return false; // 所有航点已发送
        }
    }
};

#endif
