#include "flight_controller.h"
#include <algorithm>

flight_controller::flight_controller(std::shared_ptr<quadcopter> quad_node) : Node("flight_controller_node"), quad_node(quad_node), rate(std::make_shared<rclcpp::Rate>(20)) {}

// target定点移动
void flight_controller::fly_to_target(const target& target, float timeout_sec, float stable_time_sec, int frame_rate) {
    auto quad = quad_node.lock();
    if (!quad) {
        RCLCPP_ERROR(this->get_logger(), "Quadcopter node is invalid!");
        return;
    }

    // 使用 quad_node 中的稳态时钟
    auto start_time = quad->steady_clock.now(); 
    int stable_count = 0;
    const int required_stable_frames = static_cast<int>(stable_time_sec * frame_rate);

    // 创建一个可修改的 target 副本用于更新时间戳
    class target target_cmd = target;

    do {
        // 超时检查
        if ((quad->steady_clock.now() - start_time).seconds() > timeout_sec) {
            RCLCPP_INFO(this->get_logger(), "Navigation Timeout!");
            break;
        }

        target_cmd.set_time(rclcpp::Clock().now()); // 发布时仍需使用系统时间(ROS Time)
        quad->pos_pub->publish(target_cmd.get_pose());
        
        // 稳定性检查
        if (pos_check(target_cmd)) {
            stable_count++;
        } else {
            stable_count = 0;
        }

        rate->sleep();
    } while (rclcpp::ok() && stable_count < required_stable_frames);
}

// target定点移动 (PID控制)
void flight_controller::fly_to_target_pid(const target& target, float timeout_sec, float stable_time_sec, int frame_rate) {
    auto quad = quad_node.lock();
    if (!quad) {
        RCLCPP_ERROR(this->get_logger(), "Quadcopter node is invalid!");
        return;
    }

    // PID 参数
    const float Kp_xy = 1.0f, Ki_xy = 0.1f, Kd_xy = 0.2f;
    const float Kp_z = 1.5f, Ki_z = 0.2f, Kd_z = 0.1f;
    const float max_vel_xy = 1.0f;
    const float max_vel_z = 0.5f;
    const float i_limit_xy = 0.5f;
    const float i_limit_z = 0.3f;

    // PID 状态变量
    float integral_x = 0, integral_y = 0, integral_z = 0;
    float prev_err_x = 0, prev_err_y = 0, prev_err_z = 0;
    
    int stable_count = 0;
    const int required_stable_frames = static_cast<int>(stable_time_sec * frame_rate);

    // 使用 quad_node 中的稳态时钟
    rclcpp::Time last_time = quad->steady_clock.now();
    rclcpp::Time start_time = last_time;

    while (rclcpp::ok()) {
        auto current_time = quad->steady_clock.now();
        double raw_dt = (current_time - last_time).seconds();
        double dt = raw_dt > 0
                    ? raw_dt
                    : 0.05; // 防止除0
        last_time = current_time;

        // 1. 超时检查
        if ((current_time - start_time).seconds() > timeout_sec) {
            RCLCPP_INFO(this->get_logger(), "PID Navigation Timeout!");
            break;
        }

        // 2. 稳定性检查
        if (pos_check(target)) {
            stable_count++;
            if (stable_count >= required_stable_frames) {
                RCLCPP_INFO(this->get_logger(), "Target Reached Stable (PID)");
                break;
            }
        } else {
            stable_count = 0;
        }

        // 3. 计算误差
        float err_x = target.get_x() - quad->lidar_pos->x;
        float err_y = target.get_y() - quad->lidar_pos->y;
        float err_z = target.get_z() - quad->lidar_pos->z;

        // 4. PID 计算 (X轴)
        integral_x += err_x * dt;
        integral_x = std::clamp(integral_x, -i_limit_xy, i_limit_xy);
        float deriv_x = (err_x - prev_err_x) / dt;
        float vx = Kp_xy * err_x + Ki_xy * integral_x + Kd_xy * deriv_x;
        prev_err_x = err_x;

        // PID 计算 (Y轴)
        integral_y += err_y * dt;
        integral_y = std::clamp(integral_y, -i_limit_xy, i_limit_xy);
        float deriv_y = (err_y - prev_err_y) / dt;
        float vy = Kp_xy * err_y + Ki_xy * integral_y + Kd_xy * deriv_y;
        prev_err_y = err_y;

        // PID 计算 (Z轴)
        integral_z += err_z * dt;
        integral_z = std::clamp(integral_z, -i_limit_z, i_limit_z);
        float deriv_z = (err_z - prev_err_z) / dt;
        float vz = Kp_z * err_z + Ki_z * integral_z + Kd_z * deriv_z;
        prev_err_z = err_z;

        // 5. 输出限幅
        vx = std::clamp(vx, -max_vel_xy, max_vel_xy);
        vy = std::clamp(vy, -max_vel_xy, max_vel_xy);
        vz = std::clamp(vz, -max_vel_z, max_vel_z);

        // 构造速度指令
        velocity v(vx, vy, vz);
        fly_by_velocity(v);
        
        rate->sleep();
    }
    
    // 到达目标后悬停
    velocity stop(0, 0, 0);
    fly_by_velocity(stop);
}

// 自身位置检查，distance为误差默认0.1
bool flight_controller::pos_check(const target& target, float distance) {
    auto quad = quad_node.lock();
    if (!quad) return false;

    // 注意：这里修改了 target.reached 成员，如果 target 是 const 引用，则无法修改
    // 但原逻辑中 pos_check 会修改 target->reached 状态
    // 如果传入 const target&，则不能修改 reached。
    // 建议：将 reached 状态移除出 pos_check，或者 target 传入非 const 引用，或者 reached 设为 mutable
    // 鉴于 target 类定义在 target.hpp 中，且 reached 是 public 成员，
    // 为了保持 const 正确性，这里我们只做检查，不修改 target 的状态，或者使用 const_cast (不推荐)
    // 更好的做法是让 pos_check 只负责检查，状态更新由调用者维护，或者 target 内部维护
    // 这里为了兼容，我们假设 target 的 reached 只是个标志位，不影响 const 语义（虽然严格来说是修改了）
    // 但为了编译通过，我们这里只返回 bool，不修改 target.reached
    
    float dist_sq = std::pow(quad->lidar_pos->x - target.get_x(), 2) + 
                    std::pow(quad->lidar_pos->y - target.get_y(), 2) + 
                    std::pow(quad->lidar_pos->z - target.get_z(), 2);
    
    // 使用 Throttle 防止日志刷屏，每 2000ms 打印一次距离信息用于调试
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "Distance to target: %.2f m", std::sqrt(dist_sq));

    bool is_reached = std::sqrt(dist_sq) < distance &&
                      std::abs(quad->lidar_pos->yaw - target.get_yaw()) < 0.1;
    
    // 如果确实需要修改 reached，可以使用 const_cast，但这是权宜之计
    // const_cast<class target&>(target).reached = is_reached; 
    // 或者修改 target.hpp 将 reached 设为 mutable
    
    return is_reached; 
}

// 严格检查，多维误差
bool flight_controller::pos_check(const target& target, float distance_x, float distance_y, float distance_z) {
    auto quad = quad_node.lock();
    if (!quad) return false;

    bool is_reached = std::abs(quad->lidar_pos->x - target.get_x()) < distance_x &&
                      std::abs(quad->lidar_pos->y - target.get_y()) < distance_y &&
                      std::abs(quad->lidar_pos->z - target.get_z()) < distance_z && 
                      std::abs(quad->lidar_pos->yaw - target.get_yaw()) < 0.1;
    return is_reached;
}

// velocity速度飞行，单次发布
void flight_controller::fly_by_velocity(const velocity& velocity) {
    auto quad = quad_node.lock();
    if (!quad) return;

    // velocity 是 const，不能调用非 const 方法 set_time
    // 创建副本
    class velocity vel_cmd = velocity;
    vel_cmd.set_time(rclcpp::Clock().now());
    quad->vel_pub->publish(vel_cmd.get_twist());
}

// velocity速度飞行，发布持续duration
void flight_controller::fly_by_vel_duration(const velocity& velocity, float duration) {
    auto quad = quad_node.lock();
    if (!quad) return;

    rclcpp::Time start_time = quad->steady_clock.now();
    float start_altitude = quad->lidar_pos->z; // 记录初始高度
    
    // 创建副本，避免修改传入的原始 velocity 对象
    class velocity vel_cmd = velocity;

    while (rclcpp::ok()) {
        // 每次循环重新获取锁，虽然这里 quad 局部变量一直有效，但为了保险起见
        // 其实只要 quad 没析构，引用计数就不会为0，所以这里直接用 quad 即可
        
        rclcpp::Time current_time = quad->steady_clock.now();
        if ((current_time - start_time).seconds() >= duration) break;

        // 高度保持控制 (P控制器)
        float z_error = start_altitude - quad->lidar_pos->z;
        
        // 如果误差超过死区，进行修正
        if (std::abs(z_error) > 0.1) {
            // 简单的 P 控制，限制最大垂直速度为 0.5 m/s
            float vz_correction = std::clamp(z_error * 1.0f, -0.5f, 0.5f);
            vel_cmd.set_vz(vz_correction);
        } else {
            // 在死区内，保持垂直速度为 0 (或者恢复原始指令，这里假设是定高飞行)
            vel_cmd.set_vz(0.0f);
        }

        fly_by_velocity(vel_cmd);  // 发布速度
        quad->rate->sleep();
    }
}

// 路径航点飞行，已兼容target版本
void flight_controller::fly_by_path(path* path) {
    target waypoint;
    while(rclcpp::ok()) {
        if (path->get_next_waypoint(waypoint)) {
            fly_to_target(waypoint);
        } else {
            RCLCPP_INFO(this->get_logger(), "航点已全部执行完毕");
            break;
        }
    }
}
