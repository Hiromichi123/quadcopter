#include "d435.h"

D435Node::D435Node() : Node("D435_node") {
    d435_pub = this->create_publisher<cv_tools::msg::Vision>("d435", 10);
    
    rgb_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", 1, std::bind(&D435Node::rgb_cb, this, std::placeholders::_1));
    depth_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/camera/aligned_depth_to_color/image_raw", 1, std::bind(&D435Node::depth_cb, this, std::placeholders::_1));

    // 已弃用，订阅原生d435图像(图像不对齐)
    //rgb_sub = this->create_subscription<sensor_msgs::msg::Image>("/d435/rgb", 1, std::bind(&D435Node::rgb_cb, this, std::placeholders::_1));
    //depth_sub = this->create_subscription<sensor_msgs::msg::Image>("/d435/depth", 1, std::bind(&D435Node::depth_cb, this, std::placeholders::_1));
    
    vision_msg = std::make_shared<cv_tools::msg::Vision>();
    RCLCPP_INFO(this->get_logger(), "D435 Init");
}

void D435Node::rgb_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->data.empty()) { RCLCPP_ERROR(this->get_logger(), "Empty image received"); return; }
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::cvtColor(cv_ptr->image, rgb_image, cv::COLOR_RGB2BGR);
}

void D435Node::depth_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->data.empty()) { RCLCPP_ERROR(this->get_logger(), "Empty image received"); return; }
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");
    depth_image = cv_ptr->image;
}

/* 已弃用，订阅原生d435图像(图像不对齐)
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
*/

void process(D435Node& node, cv::Mat rgb_frame, cv::Mat depth_frame) {
    if (rgb_frame.empty() || depth_frame.empty()) { return; }
    cv::Mat rgb_clone = rgb_frame.clone();
    cv::Mat depth_clone = depth_frame.clone();
    // 1. 筛选深度图中距离小于 1m 的区域
    cv::Mat mask = (depth_clone < 1.0) & (depth_clone > 0.05);  // 排除太近的无效深度

    // 2. 转成8位灰度图便于处理
    cv::Mat mask_8u;
    mask.convertTo(mask_8u, CV_8U, 255);

    // 3. 形态学操作去噪声
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
    cv::morphologyEx(mask_8u, mask_8u, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 2);
    cv::morphologyEx(mask_8u, mask_8u, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2);

    // 4. 轮廓检测
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_8u, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool valid_detection = false;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 1000) continue;  // 太小的忽略

        // 5. 在 RGB 图像上提取对应区域
        cv::Rect bound = cv::boundingRect(contour);
        cv::rectangle(rgb_clone, bound, cv::Scalar(255, 0, 0), 2);  // 框出候选区域

        // 6. 红色检测（在ROI区域）
        cv::Mat roi = rgb_frame(bound);
        cv::Mat hsv;
        cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

        cv::Mat red_mask1, red_mask2, red_mask;
        cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), red_mask1);
        cv::inRange(hsv, cv::Scalar(160, 120, 70), cv::Scalar(180, 255, 255), red_mask2);
        cv::bitwise_or(red_mask1, red_mask2, red_mask);

        int red_pixels = cv::countNonZero(red_mask);
        if (red_pixels > 1000) {
            // 红色轮廓中心（相对于整个图像）
            cv::Moments m = cv::moments(red_mask, true);
            if (m.m00 == 0) continue;
            cv::Point2f center_in_roi(m.m10/m.m00, m.m01/m.m00);
            cv::Point2f center_in_image(bound.x + center_in_roi.x, bound.y + center_in_roi.y);

            // 获取中心点深度值, 并计算平均深度
            std::vector<float> valid_depths;
            int sample_step = 5;  // 采样步长，避免采样过多点
            float total_depth = 0.0f;
            int valid_count = 0;
            std::vector<cv::Point> red_contour_points;
            cv::findNonZero(red_mask, red_contour_points);
            for (size_t i = 0; i < red_contour_points.size(); i += sample_step) {
                cv::Point pt = red_contour_points[i];
                cv::Point2f image_point(bound.x + pt.x, bound.y + pt.y);
                float depth = depth_clone.at<float>(image_point.y, image_point.x);
                if (!std::isnan(depth) && depth > 0.05 && depth < 1.0) {
                    valid_depths.push_back(depth);
                    total_depth += depth;
                    valid_count++;
                }
            }
            if (valid_count == 0) continue;
            // 计算中位数深度（更抗噪声）
            std::sort(valid_depths.begin(), valid_depths.end());
            float median_depth = valid_depths[valid_depths.size()/2];
            
            valid_detection = true;
            node.vision_msg->is_red_detected = true;
            node.vision_msg->later_error = center_in_image.x - rgb_frame.cols/2; // 中心点与图像中心的水平偏差
            node.vision_msg->distance = median_depth;  // 深度值
            
            // 可视化
            cv::circle(rgb_clone, center_in_image, 5, cv::Scalar(0, 255, 0), -1);  // 绿色点标记中心
            cv::putText(rgb_clone, "Red_Pole", bound.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
            std::string dist_text = "Dist: " + std::to_string(median_depth) + "m";
            cv::putText(rgb_clone, dist_text, cv::Point(bound.x, bound.y-10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }
    }
    if (!valid_detection) {
        node.vision_msg->is_red_detected = false;
        node.vision_msg->later_error = 0.0;
        node.vision_msg->distance = 0.0;
    }

    cv::imshow("RGB", rgb_clone);
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
