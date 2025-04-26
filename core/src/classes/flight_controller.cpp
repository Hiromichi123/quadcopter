#include "flight_controller.h"

flight_controller::flight_controller(std::shared_ptr<quadcopter> quad_node) : Node("flight_controller_node"), quad_node(quad_node), rate(std::make_shared<rclcpp::Rate>(20)) {}

// target定点移动
void flight_controller::fly_to_target(target* target) {
    do {
        target->set_time(rclcpp::Clock().now());
        quad_node->pos_pub->publish(target->get_pose());
        rate->sleep();
    } while (rclcpp::ok() && !pos_check(target));
}

// 自身位置检查，distance为误差默认0.1
bool flight_controller::pos_check(target* target, float distance) {
    return target->reached || (target->reached = std::sqrt(std::pow(quad_node->lidar_pos->x - target->get_x(), 2) +
                                                std::pow(quad_node->lidar_pos->y - target->get_y(), 2) + 
                                                std::pow(quad_node->lidar_pos->z - target->get_z(), 2)) < distance &&
                                                std::abs(quad_node->lidar_pos->yaw - target->get_yaw()) < 0.1);}

// 严格检查，多维误差
bool flight_controller::pos_check(target* target, float distance_x, float distance_y, float distance_z) {
    return target->reached || (target->reached = std::abs(quad_node->lidar_pos->x - target->get_x()) < distance_x &&
                                                std::abs(quad_node->lidar_pos->y - target->get_y()) < distance_y &&
                                                std::abs(quad_node->lidar_pos->z - target->get_z()) < distance_z && 
                                                std::abs(quad_node->lidar_pos->yaw - target->get_yaw()) < 0.1);}

// velocity速度飞行，单次发布
void flight_controller::fly_by_velocity(velocity* velocity) {
    velocity->set_time(rclcpp::Clock().now());
    quad_node->vel_pub->publish(velocity->get_twist());
}

// velocity速度飞行，发布持续duration
void flight_controller::fly_by_vel_duration(velocity* velocity, float duration) {
    rclcpp::Time start_time = rclcpp::Clock().now();
    rclcpp::Time current_time = rclcpp::Clock().now();
    auto elapsed_time = current_time - start_time;
    float start_altitude = quad_node->lidar_pos->z; // 记录初始高度

    while (elapsed_time.seconds() < duration) {
        current_time = rclcpp::Clock().now();
        elapsed_time = current_time - start_time;

        // 修改z轴反馈
        if (std::abs(start_altitude - quad_node->lidar_pos->z) > 0.1) {
            velocity->set_vx(start_altitude - quad_node->lidar_pos->z);
        }

        fly_by_velocity(velocity);  // 发布速度
        quad_node->rate->sleep();
    }
}

// 路径航点飞行，已兼容target版本
void flight_controller::fly_by_path(path* path) {
    target waypoint;
    while(rclcpp::ok()) {
        if (path->get_next_waypoint(waypoint)) {
            fly_to_target(&waypoint);
        } else {
            RCLCPP_INFO(this->get_logger(), "航点已全部执行完毕");
            break;
        }
    }
}
