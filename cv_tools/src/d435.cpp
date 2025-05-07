#include "d435.h"

D435Node::D435Node() : Node("D435_node") {
    d435_pub = this->create_publisher<cv_tools::msg::Vision>("d435", 10);
    
    rgb_sub = this->create_subscription<sensor_msgs::msg::Image>("/d435/rgb", 1, std::bind(&D435Node::rgb_cb, this, std::placeholders::_1));
    depth_sub = this->create_subscription<sensor_msgs::msg::Image>("/d435/depth", 1, std::bind(&D435Node::depth_cb, this, std::placeholders::_1));
    
    vision_msg = std::make_shared<cv_tools::msg::Vision>();
    RCLCPP_INFO(this->get_logger(), "D435 Init");
}

void D435Node::rgb_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->data.empty()) { RCLCPP_ERROR(this->get_logger(), "Empty image received"); return; }
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    rgb_image = cv_ptr->image;
}

void D435Node::depth_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->data.empty()) { RCLCPP_ERROR(this->get_logger(), "Empty image received"); return; }
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "32FC1");
    depth_image = cv_ptr->image;
}

void process(D435Node node, cv::Mat rgb_frame, cv::Mat depth_frame) {
    // 1. 筛选深度图中距离小于 1m 的区域
    cv::Mat mask = (depth < 1.0) & (depth > 0.05);  // 排除太近的无效深度

    // 2. 转成8位灰度图便于处理
    cv::Mat mask_8u;
    mask.convertTo(mask_8u, CV_8U, 255);

    // 3. 形态学操作去噪声
    cv::morphologyEx(mask_8u, mask_8u, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2);
    cv::morphologyEx(mask_8u, mask_8u, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 2);

    // 4. 轮廓检测
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_8u, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool valid_detection = false;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 1000) continue;  // 太小的忽略

        // 5. 在 RGB 图像上提取对应区域
        cv::Rect bound = cv::boundingRect(contour);
        cv::rectangle(rgb, bound, cv::Scalar(255, 0, 0), 2);  // 蓝色框出候选区域

        // 6. 红色检测（在ROI区域）
        cv::Mat roi = rgb(bound);
        cv::Mat hsv;
        cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

        cv::Mat red_mask1, red_mask2, red_mask;
        cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), red_mask1);
        cv::inRange(hsv, cv::Scalar(160, 120, 70), cv::Scalar(180, 255, 255), red_mask2);
        cv::bitwise_or(red_mask1, red_mask2, red_mask);

        int red_pixels = cv::countNonZero(red_mask);
        if (red_pixels > 1000) {
            cv::putText(rgb, "red_gan", bound.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
            valid_detection = true;
        }
    }
    if (!valid_detection) { return; }

    node.d435_pub->publish(*node.vision_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<D435Node>();
    rclcpp::Rate rate(30);
    while (rclcpp::ok()) {
        process(*node, rgb_image, depth_image);
        rclcpp::spin_some(node);
        rate.sleep();
    }
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
