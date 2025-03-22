#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <vision/msg/vision.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "cv_tools.hpp"

class VisionPubNode : public rclcpp::Node {
    public:
        VisionPubNode() : Node("vision_pub") {
            vision_pub = this->create_publisher<vision::msg::Vision>("vision", 10);
            // frame_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>("frame", 10); // 发布压缩图像调试
            frame_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/ground", 1, std::bind(&VisionPubNode::ground_callback, this, std::placeholders::_1));
            bridge = std::make_shared<cv_bridge::CvBridge>();
            cv_tools = std::make_shared<CVTools>(this);  // 创建工具类实例
            RCLCPP_INFO(this->get_logger(), "Init complete");
        }
    
    private:
        void ground_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            if (msg->data.empty()) { RCLCPP_ERROR(this->get_logger(), "Empty image received"); return; }// 检查图像是否为空
            try {
                cv::Mat cv_image = bridge->imgmsg_to_cv2(msg, "bgr8");
                process(cv_image);
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error in processing: %s", e.what());
            }
        }
    
        void process(cv::Mat frame) {
            cv::Mat frame1, frame2, frame3;
            frame1 = cv_tools->red_circle_detect(frame); // 红色圆检测
            frame2 = cv_tools->yellow_square_detect(frame); // 黄色方检测
            frame3 = cv_tools->line_detect(frame); // 霍夫直线检测            
            
            vision_pub->publish(vision_msg);
            // frame_pub->publish(*bridge->cv2_to_compressed_imgmsg(frame1)); // 发布压缩图像调试
        }
    
        rclcpp::Publisher<vision::msg::Vision>::SharedPtr vision_pub;
        // rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr frame_pub1;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_sub;
        std::shared_ptr<cv_bridge::CvBridge> bridge;
        std::shared_ptr<CVTools> cv_tools;
        vision::msg::Vision vision_msg;
    };
    
    int main(int argc, char** argv) {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<VisionPubNode>();
        rclcpp::spin(node);
        cv::destroyAllWindows();
        rclcpp::shutdown();
        return 0;
    }
    