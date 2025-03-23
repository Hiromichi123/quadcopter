#include "vision_node.h"

VisionNode::VisionNode() : Node("vision_pub") {
    vision_pub = this->create_publisher<cv_tools::msg::VisionMsg>("vision", 10);
    // frame_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>("frame", 10); // 发布压缩图像调试
    frame_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/ground", 1, std::bind(&VisionNode::ground_callback, this, std::placeholders::_1));
    bridge = std::make_shared<cv_bridge::CvBridge>();
    cv_tools = std::make_shared<CVTools>(this);  // 创建工具类实例
    RCLCPP_INFO(this->get_logger(), "Init complete");
}

void VisionNode::fill_circle_msg(bool is_detected, int center_x_error, int center_y_error) {
    vision_msg.is_circle_detected = is_detected;
    vision_msg.center_x2_error = center_x_error;
    vision_msg.center_y2_error = center_y_error;
}

void VisionNode::fill_square_msg(bool is_detected, int center_x_error, int center_y_error) {
    vision_msg.is_square_detected = is_detected;
    vision_msg.center_x1_error = center_x_error;
    vision_msg.center_y1_error = center_y_error;
}

void VisionNode::fill_line_msg(bool is_detected, int lateral_error, double angle_error) {
    vision_msg.is_line_detected = is_detected;
    vision_msg.lateral_error = lateral_error;
    vision_msg.angle_error = angle_error;
}

void VisionNode::ground_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->data.empty()) { RCLCPP_ERROR(this->get_logger(), "Empty image received"); return; }// 检查图像是否为空
    try {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");  
        cv::Mat cv_image = cv_ptr->image;
        process(cv_image);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in processing: %s", e.what());
    }
}

void VisionNode::process(cv::Mat frame) {
    cv::Mat frame1, frame2, frame3;
    frame1 = cv_tools->red_circle_detect(frame); // 红色圆检测
    frame2 = cv_tools->yellow_square_detect(frame); // 黄色方检测
    frame3 = cv_tools->line_detect(frame); // 霍夫直线检测            
    
    vision_pub->publish(vision_msg);
    // frame_pub->publish(*bridge->cv2_to_compressed_imgmsg(frame1)); // 发布压缩图像调试
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionNode>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
