#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_tools/msg/vision.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

cv::Mat rgb_image, depth_image;

class D435Node : public rclcpp::Node {
public:
    D435Node();

    rclcpp::Publisher<cv_tools::msg::Vision>::SharedPtr d435_pub;
    std::shared_ptr<cv_tools::msg::Vision> vision_msg;

private:
    void rgb_cb(const sensor_msgs::msg::Image::SharedPtr msg);
    void depth_cb(const sensor_msgs::msg::Image::SharedPtr msg);

    // rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr frame_pub1;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
};

void process(cv::Mat frame);
