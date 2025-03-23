#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_tools/msg/vision_msg.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "cv_tools.hpp"

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("vision_node");

    void fill_circle_msg(bool is_detected, int center_x_error, int center_y_error);
    void fill_square_msg(bool is_detected, int center_x_error, int center_y_error);
    void fill_line_msg(bool is_detected, int lateral_error, double angle_error);
    
private:
    void ground_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void process(cv::Mat frame);

    rclcpp::Publisher<cv_tools::msg::VisionMsg>::SharedPtr vision_pub;
    // rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr frame_pub1;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_sub;
    std::shared_ptr<cv_bridge::CvImage> bridge;
    std::shared_ptr<cv::functions::CVTools> cv_tools;
    cv_tools::msg::VisionMsg vision_msg;
};
