#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class D435Node : public rclcpp::Node {
public:
    D435Node() : Node("d435_node") {
        rgb_pub = this->create_publisher<sensor_msgs::msg::Image>("/d435/rgb", 10);
        depth_pub = this->create_publisher<sensor_msgs::msg::Image>("/d435/depth", 10);

        // 配置RealSense流
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
        pipe.start(cfg); // 启动管道

        // cv_bridge图像转换
        rgb = std::make_shared<cv_bridge::CvImage>();
        depth = std::make_shared<cv_bridge::CvImage>();
        rgb->encoding = sensor_msgs::image_encodings::BGR8;
        depth->encoding = sensor_msgs::image_encodings::TYPE_16UC1;

        // timer采集与发布
        timer = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&D435Node::publish_images, this)
        );
    }

private:
    void publish_images() {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();

        if (!color_frame || !depth_frame) return;

        // 转换OpenCV格式, 转换为ROS2消息, 发布
        cv::Mat color_image(cv::Size(1280, 720), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        rgb->image = color_image;
        depth->image = depth_image;

        rgb_pub->publish(*(rgb->toImageMsg()));
        depth_pub->publish(*(depth->toImageMsg()));
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
    rs2::pipeline pipe;
    std::shared_ptr<cv_bridge::CvImage> rgb;
    std::shared_ptr<cv_bridge::CvImage> depth;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<D435Node>());
    rclcpp::shutdown();
    return 0;
}
