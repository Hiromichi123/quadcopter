#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <vision/msg/vision.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "cv_tools.hpp"
#include "cv_pipeline.hpp"

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
            if (msg->data.empty()) { // 检查图像是否为空
                RCLCPP_ERROR(this->get_logger(), "Empty image received");
                return;
            }
            try {
                cv::Mat cv_image = bridge->imgmsg_to_cv2(msg, "bgr8");
                process(cv_image);
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error convert image: %s", e.what());
            }
        }
    
        void process(cv::Mat frame) {
            try {
                frame1 = frame.clone();
                cvPipeline red_circle_pipe; // 红色圆形检测管道
                red_circle_pipe.do(GaussianBlur, 5)
                              .do(Canny, 100, 200)
                              .process(&frame1);
                // cv::imshow("red", frame1);
                
                frame2 = frame.clone();
                cvPipeline yellow_square_pipe; // 黄色矩形检测管道
                yellow_square_pipe.do(GaussianBlur, 5)
                                 .do(Canny, 100, 200)
                                 .process(&frame2);
                // cv::imshow("yellow", frame2);
                
    
                frame3 = frame(cv::Range::all(), cv::Range(frame.cols / 6, frame.cols - frame.cols / 6));
                cvPipeline line_detect_pipe; // 直线检测管道
                line_detect_pipe.do(cvtColor, cv::COLOR_BGR2GRAY)
                               .do(threshold, 100, 255, cv::THRESH_BINARY)
                               .process(&frame3);
                // cv::imshow("line", frame3);
                cv::Mat hl_copy = cv_tools->line_detect(frame3);  // 霍夫直线
                // cv::imshow("霍夫直线效果", hl_copy);
                // cv::waitKey(1);
                
                vision_pub->publish(vision_msg);
                // frame_pub->publish(*bridge->cv2_to_compressed_imgmsg(frame1)); // 发布压缩图像调试

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error occurred: %s", e.what());
            }
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
    