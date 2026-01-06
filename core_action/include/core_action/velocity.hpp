#ifndef CORE_ACTION_VELOCITY_HPP
#define CORE_ACTION_VELOCITY_HPP

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace core_action {
    // 速度类
    class Velocity {
    public:
        Velocity(float v_x, float v_y, float v_z, 
                float v_yaw = 0.0f, float v_pitch = 0.0f, float v_roll = 0.0f) {
            twist_stamped.twist.linear.x = v_x;
            twist_stamped.twist.linear.y = v_y;
            twist_stamped.twist.linear.z = v_z;
            twist_stamped.twist.angular.z = v_yaw;
            twist_stamped.twist.angular.y = v_pitch;
            twist_stamped.twist.angular.x = v_roll;
        }

        // 从 TwistStamped 快速构造，必须显式调用
        explicit Velocity(const geometry_msgs::msg::TwistStamped& twist) {
            twist_stamped = twist;
        }

        // 获取成员
        float get_vx() const { return twist_stamped.twist.linear.x; }
        float get_vy() const { return twist_stamped.twist.linear.y; }
        float get_vz() const { return twist_stamped.twist.linear.z; }
        float get_vyaw() const { return twist_stamped.twist.angular.z; }
        float get_vpitch() const { return twist_stamped.twist.angular.y; }
        float get_vroll() const { return twist_stamped.twist.angular.x; }

        // 获取发布所需的TwistStamped结构体
        geometry_msgs::msg::TwistStamped get_twist() const { return twist_stamped; }

        // 设置成员
        void set_vx(float vx) { twist_stamped.twist.linear.x = vx; }
        void set_vy(float vy) { twist_stamped.twist.linear.y = vy; }
        void set_vz(float vz) { twist_stamped.twist.linear.z = vz; }
        void set_vyaw(float vyaw) { twist_stamped.twist.angular.z = vyaw; }
        void set_vpitch(float vpitch) { twist_stamped.twist.angular.y = vpitch; }
        void set_vroll(float vroll) { twist_stamped.twist.angular.x = vroll; }

        // 设置时间戳
        void set_time(rclcpp::Time time) { twist_stamped.header.stamp = time; }

        // 手动/隐式转换
        operator geometry_msgs::msg::TwistStamped&() { return twist_stamped; }
        operator const geometry_msgs::msg::TwistStamped&() const { return twist_stamped; }

    private:
        geometry_msgs::msg::TwistStamped twist_stamped;
    };
}

#endif // CORE_ACTION_VELOCITY_HPP
