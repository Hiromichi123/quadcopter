#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class LidarToPx4Bridge : public rclcpp::Node {
public:
  LidarToPx4Bridge() : Node("lidar_to_px4_bridge") {
    // 订阅 Odometry 消息
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/Odometry", 10, std::bind(&LidarToPx4Bridge::odomCallback, this, std::placeholders::_1));

    // 发布给 PX4 飞控
    vision_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto vision_pose = geometry_msgs::msg::PoseStamped();
    // 设置时间戳
    vision_pose.header.stamp = this->now();
    vision_pose.header.frame_id = "map";

    // 位置
    vision_pose.pose.position.x = msg->pose.pose.position.x;
    vision_pose.pose.position.y = msg->pose.pose.position.y;
    vision_pose.pose.position.z = msg->pose.pose.position.z;

    // 姿态（四元数）
    vision_pose.pose.orientation = msg->pose.pose.orientation;

    vision_pose_pub->publish(vision_pose);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarToPx4Bridge>());
  rclcpp::shutdown();
  return 0;
}