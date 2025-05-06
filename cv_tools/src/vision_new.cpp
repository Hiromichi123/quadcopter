#include "vision_new.h"

VisionNode::VisionNode() : Node("vision_pub") {
    vision_pub = this->create_publisher<cv_tools::msg::VisionMsg>("vision", 10);
    // frame_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>("frame", 10); // 发布压缩图像调试
    frame_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/ground", 1, std::bind(&VisionNode::ground_callback, this, std::placeholders::_1));
    vision_msg = std::make_shared<cv_tools::msg::VisionMsg>(); // 创建消息实例
    cv_tools = std::make_shared<CVTools>(vision_msg);  // 创建工具类实例
    RCLCPP_INFO(this->get_logger(), "Init complete");
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
    cv::Mat frame1 = frame.clone(); // 深拷贝图像, 无效占位操作
    vision_pub->publish(*vision_msg);
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
