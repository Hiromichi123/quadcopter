#ifndef CORE_ACTION_PATH_HPP
#define CORE_ACTION_PATH_HPP

#include <vector>
#include <iostream>
#include "core_action/target.hpp"

namespace core_action {

// 路径类 - 支持 Target 对象
class Path {
public:
    size_t current_index = 0;
    std::vector<Target> waypoints;
    
    // 添加航点 (Target 版本)
    void add_waypoint(const Target& waypoint) {
        waypoints.push_back(waypoint);
    }

    // 添加航点 (直接参数版本)
    void add_waypoint(float x, float y, float z, float yaw = 0.0f) {
        waypoints.emplace_back(x, y, z, yaw);
    }

    // 删除航点
    void remove_waypoint(size_t erase_num) {
        if (waypoints.empty()) {
            std::cerr << "Path: 路径已经为空！" << std::endl;
            return;
        }

        if (erase_num < waypoints.size()) {
            waypoints.erase(waypoints.begin() + erase_num);
        } else {
            std::cerr << "Path: 非法航点索引: " << erase_num << std::endl;
        }
    }

    // 获取下一个航点
    bool get_next_waypoint(Target& waypoint) {
        if (current_index < waypoints.size()) {
            waypoint = waypoints[current_index++];
            return true;
        } else {
            current_index = 0;
            return false; // 所有航点已发送
        }
    }

    // 重置索引
    void reset() {
        current_index = 0;
    }

    // 获取航点数量
    size_t size() const {
        return waypoints.size();
    }

    // 清空所有航点
    void clear() {
        waypoints.clear();
        current_index = 0;
    }

    // 检查是否为空
    bool empty() const {
        return waypoints.empty();
    }
};

} // namespace core_action

#endif // CORE_ACTION_PATH_HPP
