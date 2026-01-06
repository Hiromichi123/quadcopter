#ifndef CORE_ACTION_PATH_HPP
#define CORE_ACTION_PATH_HPP

#include <vector>
#include <iostream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "core_action/target.hpp"

namespace core_action {
    // 路径类
    class Path {
    public:
        size_t current_index = 0;
        std::vector<Target> waypoints;

        // 注册航点（PoseStamped 版本）
        auto add_waypoint(const geometry_msgs::msg::PoseStamped& waypoint) -> bool {
            waypoints.emplace_back(waypoint);
            return true;
        }

        // 注册航点 (Target 版本)
        auto add_waypoint(const Target& waypoint) -> bool {
            waypoints.push_back(waypoint);
            return true;
        }

        // 注册航点 (直接参数版本)
        auto add_waypoint(float x, float y, float z, float yaw = 0.0f) -> bool{
            waypoints.emplace_back(x, y, z, yaw);
            return true;
        }

        // 删除航点
        auto remove_waypoint(size_t erase_num) -> bool {
            if (waypoints.empty()) {
                std::cerr << "Path: 路径已经为空！" << std::endl;
                return false;
            }

            if (erase_num < waypoints.size()) {
                waypoints.erase(waypoints.begin() + erase_num);
            } else {
                std::cerr << "Path: 非法航点索引: " << erase_num << std::endl;
                return false;
            }
            return true;
        }

        // 获取下一个航点
        auto get_next_waypoint(Target& waypoint) -> bool {
            if (current_index < waypoints.size()) {
                waypoint = waypoints[current_index++];
                return true;
            } else {
                current_index = 0;
                return false; // 所有航点已发送
            }
        }

        // 重置索引
        auto reset() -> bool {
            current_index = 0;
            return true;
        }

        // 获取航点数量
        auto size() const -> size_t {
            return waypoints.size();
        }

        // 清空所有航点
        auto clear() -> bool {
            waypoints.clear();
            current_index = 0;
            return true;
        }

        // 检查是否为空
        auto empty() const -> bool {
            return waypoints.empty();
        }
    };
}

#endif // CORE_ACTION_PATH_HPP
