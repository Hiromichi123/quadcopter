#include<quadcopter.h>
#include "flight_controller.h"

quadcopter::quadcopter() : Node("quad_node") {
    rate = std::make_shared<rclcpp::Rate>(20.0);

    lidar_pos = std::make_shared<ros2_tools::msg::LidarPose>();
    lidar_sub = this->create_subscription<ros2_tools::msg::LidarPose>("lidar_data", 10, std::bind(&quadcopter::lidar_pose_cb, this, std::placeholders::_1));
    current_state = std::make_shared<mavros_msgs::msg::State>();
    state_sub = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, std::bind(&quadcopter::state_cb, this, std::placeholders::_1));
    vision_msg = std::make_shared<vision::msg::Vision>();
    vision_sub = this->create_subscription<vision::msg::Vision>("vision", 10, std::bind(&quadcopter::vision_sub_cb, this, std::placeholders::_1));

    pos_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
    vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    arming_client = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    command_client = this->create_client<mavros_msgs::srv::CommandLong>("mavros/cmd/command");
    set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    RCLCPP_INFO(this->get_logger(), "飞行器初始化完成");
}

// 初始化化飞行控制类
void quadcopter::flight_ctrl_init() {
    flight_ctrl = std::make_shared<flight_controller>(std::static_pointer_cast<quadcopter>(shared_from_this()));
    RCLCPP_INFO(this->get_logger(), "飞行控制类初始化完成");
}

// 注册shutdown回调，并创建spin线程处理回调
void quadcopter::start_spin_thread() {
    spin_thread = std::make_shared<std::thread>([this]() {
        while (rclcpp::ok()) {
            rclcpp::spin_some(shared_from_this());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    rclcpp::on_shutdown([this]() {
        if (spin_thread && spin_thread->joinable()) {
            spin_thread->join();
        }
    });
}

// 起飞前检查
void quadcopter::pre_flight_checks_loop() {
    mavros_msgs::srv::SetMode::Request offb_set_mode;
    offb_set_mode.custom_mode = "OFFBOARD";
    auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    *mode_request = offb_set_mode;

    mavros_msgs::srv::CommandBool::Request arm_cmd;
    arm_cmd.value = true;
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    *arm_request = arm_cmd;

    // 起飞点预发布
    target simp(0, 0, 0.5, 0);
    for (int i = 0; i < 20; ++i) {
        simp.set_time(this->now());
        pos_pub->publish(simp.get_pose());
        rate->sleep();
    }

    rclcpp::Time last_request = this->now();
    while (rclcpp::ok()) {
        simp.set_time(this->now());
        pos_pub->publish(simp.get_pose());

        if (!current_state->armed && (this->now() - last_request > rclcpp::Duration::from_seconds(1.0))) {
            if (arming_client->async_send_request(arm_request).valid()) { // 定时检查是否解锁
                RCLCPP_INFO(this->get_logger(), "arming...");
            }
            last_request = this->now();
        } else if (current_state->mode != "OFFBOARD" && (this->now() - last_request > rclcpp::Duration::from_seconds(1.0))) {
            if (set_mode_client->async_send_request(mode_request).valid()) { // 定时尝试OFFBOARD
                RCLCPP_INFO(this->get_logger(), "armed and OFFBOARDING...");
            }
            last_request = this->now();
        } else if (current_state->armed && current_state->mode == "OFFBOARD") { 
            RCLCPP_INFO(this->get_logger(), "armed and OFFBOARD success!");
            flight_ctrl->fly_to_target(&simp); // 起飞点
            break;
        }
        rate->sleep();
    }
}

// 主循环
void quadcopter::main_loop() {
    /* -----------------------示例-------------------------------
    -------------------------------------------------------------
    target first_point(1.0, 0, 0.5, 0); // 第一个目标点
    velocity first_vel(0.1, 0, 0); // 第一段速度
    path path1; // 第一段路径
    path1.add_waypoint(target(1.0, 0, 0.5, 0)); // 被移除
    path1.add_waypoint(target(0, 1.0, 0.5, 0));
    path1.add_waypoint(target(-1.0, 0, 2.5, 0));
    path1.remove_waypoint(0); // 移除第一个点

    int state = 0;
    while (rclcpp::ok()) {
        if(state == 0) {
            // 三种移动测试
            // 第一种：定点移动
            flight_ctrl->fly_to_target(&first_point);
            RCLCPP_INFO(this->get_logger(), "到达第一个点");
            // 第二种：速度移动
            flight_ctrl->fly_by_vel_duration(&first_vel, 5.0);
            RCLCPP_INFO(this->get_logger(), "第一次速度飞行结束");
            // 第三种：航点移动
            flight_ctrl->fly_by_path(&path1);
            RCLCPP_INFO(this->get_logger(), "第一段路径飞行结束");
            state = 1;
        }
        rate->sleep();
    }
    ------------------------------------------------------------
    --------------------------示例----------------------------*/

    int flag = 0;
    float default_altitude = 1.5;
    bool is_complete_cast = false;
    target first_point(0.0, 0.0, 1.5, 0.0);
    target tar1(0.0, 0.0, 0.0, 0.0);
    velocity vel1(0.1, 0.0, 0.0, 0.0);
    velocity vel2(0.15, 0.0, 0.0, 0.0);
    while (rclcpp::ok()) {
        switch (flag) {
            case 0:
                flight_ctrl->fly_to_target(&first_point);
                RCLCPP_INFO(this->get_logger(), "到达指定高度");
                flag = 1;
                RCLCPP_INFO(this->get_logger(), "前进");
                break;
            case 1:
                flight_ctrl->fly_by_velocity(&vel1);
                if (vision_msg->is_line_detected) {
                    RCLCPP_INFO(this->get_logger(), "发现直线");
                    flag = 2;
                    RCLCPP_INFO(this->get_logger(), "巡线");
                }
                break;
            case 2:
                vel2.set_vy(vision_msg->lateral_error/-1000.0);
                vel2.set_vyaw(vision_msg->angle_error/5);
                if (default_altitude - z > 0.05) { vel2.set_vz(default_altitude - z); } 
                else if (default_altitude - z < -0.05) { vel2.set_vz(default_altitude - z); }
                flight_ctrl->fly_by_velocity(&vel2);
                if (vision_msg->is_square_detected && !is_complete_cast) {
                    // 记录位置
                    tar1.set_x(x);
                    tar1.set_y(y);
                    tar1.set_z(z);
                    tar1.set_yaw(yaw);
                    RCLCPP_INFO(this->get_logger(), "发现形状, 记录位置: x=%f, y=%f, z=%f, yaw=%f", x, y, z, yaw);
                    flag = 3;
                    RCLCPP_INFO(this->get_logger(), "进入校准");
                }
                if (vision_msg->is_circle_detected && is_complete_cast) {
                    RCLCPP_INFO(this->get_logger(), "发现降落区域");
                    flag = 5;
                    RCLCPP_INFO(this->get_logger(), "进入校准降落");
                }
                break;
            case 3:
                vel2.set_vx(vision_msg->center_x1_error/-1000.0);
                vel2.set_vy(vision_msg->center_y1_error/-1000.0);
                vel2.set_vyaw(0.0);
                if (default_altitude - z > 0.05) { vel2.set_vz(default_altitude - z); } 
                else if (default_altitude - z < -0.05) { vel2.set_vz(default_altitude - z); }
                flight_ctrl->fly_by_velocity(&vel2);
                if (std::abs(vision_msg->center_x1_error) < 20 && std::abs(vision_msg->center_y1_error) < 20) {
                    RCLCPP_INFO(this->get_logger(), "投掷");
                    // 投掷
                    is_complete_cast = true;
                    flag = 4;
                    RCLCPP_INFO(this->get_logger(), "返回巡线");
                }
                break;
            case 4:
                flight_ctrl->fly_to_target(&tar1);
                vel2.set_vx(0.15);
                RCLCPP_INFO(this->get_logger(), "继续巡线");
                flag = 2;
                break;
            case 5:
                vel2.set_vx(vision_msg->center_x2_error/-1000.0);
                vel2.set_vy(vision_msg->center_y2_error/-1000.0);
                vel2.set_vyaw(0.0);
                if (default_altitude - z > 0.05) { vel2.set_vz(0.03); } 
                else if (default_altitude - z < -0.05) { vel2.set_vz(-0.03); }
                flight_ctrl->fly_by_velocity(&vel2);
                if (std::abs(vision_msg->center_x2_error) < 20 && std::abs(vision_msg->center_y2_error) < 20) {
                    RCLCPP_INFO(this->get_logger(), "降落");
                    vel2.set_vx(0.0);
                    vel2.set_vy(0.0);
                    vel2.set_vz(-0.2);
                    flight_ctrl->fly_by_vel_duration(&vel2, 5.0);
                    RCLCPP_INFO(this->get_logger(), "降落完成");
                    flag = 6;
                }
                break;
        }
        rate->sleep();
    }
}

// 初始化quad节点控制流程
void quadcopter::quad_init() {
    flight_ctrl_init();
    start_spin_thread();
    pre_flight_checks_loop();
    main_loop();
}

// lidar数据回调
void quadcopter::lidar_pose_cb(const ros2_tools::msg::LidarPose::SharedPtr msg) {
    lidar_pos = msg;
    x = msg->x;
    y = msg->y;
    z = msg->z;
    yaw = msg->yaw;
}

// mavros状态回调
void quadcopter::state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state = msg;
}

// vision数据回调
void quadcopter::vision_sub_cb(const vision::msg::Vision::SharedPtr msg) {
    vision_msg = msg;
}

// 主程序入口
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto quad_node = std::make_shared<quadcopter>();
    quad_node->quad_init();
    rclcpp::shutdown();
    return 0;
}