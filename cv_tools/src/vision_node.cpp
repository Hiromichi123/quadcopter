#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <vision/msg/vision.hpp>
#include <cv_bridge/cv_bridge.h>
#include "cv_tools.hpp"

class VisionPubNode : public rclcpp::Node {
    public:
        VisionPubNode() : Node("vision_pub") {
            vision_pub_ = this->create_publisher<vision::msg::Vision>("vision", 10);
            // frame_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>("frame", 10); // 发布压缩图像调试
    
            frame_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/ground", 1, std::bind(&VisionPubNode::ground_callback, this, std::placeholders::_1));
            
            bridge = std::make_shared<cv_bridge::CvBridge>();
            cv_tools = std::make_shared<CVTools>(this);  // 创建工具类实例
    
            msg_.is_line_detected = false;
            msg_.lateral_error = 0;
            msg_.angle_error = 0.0;
            msg_.is_square_detected = false;
            msg_.center_x1_error = 0;
            msg_.center_y1_error = 0;
            msg_.is_circle_detected = false;
            msg_.center_x2_error = 0;
            msg_.center_y2_error = 0;
    
            RCLCPP_INFO(this->get_logger(), "Init complete");
        }
    
    private:
        void ground_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            try {
                cv::Mat cv_image = bridge_->imgmsg_to_cv2(msg, "bgr8");
                process(cv_image);
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error convert image: %s", e.what());
            }
        }
    
        void process(cv::Mat frame) {
            try {
                cv::Mat detect_copy2 = cv_tools->red_circle_detect(frame);  // 红色圆形检测
                // cv::imshow("red", detect_copy2);
                
                cv::Mat detect_copy1 = cv_tools->yellow_square_detect(frame);  // 矩形检测
                // cv::imshow("yellow", detect_copy1);
                // cv::waitKey(1);
                
                // frame_pub->publish(*bridge->cv2_to_compressed_imgmsg(detect_copy1));
    
                frame = frame(cv::Range::all(), cv::Range(frame.cols / 6, frame.cols - frame.cols / 6));
                cv::Mat gray_frame;
                cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
                
                cv::Mat thresh_frame;
                cv::threshold(gray_frame, thresh_frame, 100, 255, cv::THRESH_BINARY);
                
                cv::Mat hl_copy = cv_tools->line_detect(thresh_frame);  // 霍夫直线
                // cv::imshow("霍夫直线效果", hl_copy);
                
                vision_pub_->publish(msg_);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error occurred: %s", e.what());
            }
        }
    
        rclcpp::Publisher<vision::msg::Vision>::SharedPtr vision_pub;
        // rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr frame_pub1;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_sub;
        std::shared_ptr<cv_bridge::CvBridge> bridge;
        std::shared_ptr<CVTools> cv_tools;
        vision::msg::Vision msg;
    };
    
    int main(int argc, char** argv) {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<VisionPubNode>();
        rclcpp::spin(node);
        cv::destroyAllWindows();
        rclcpp::shutdown();
        return 0;
    }
    