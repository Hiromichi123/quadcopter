#ifndef CV_TOOLS_HPP
#define CV_TOOLS_HPP
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cmath>
#include <cv_tools/msg/vision_msg.hpp>
#include "cv_pipeline.hpp"
#include "cv_functions.hpp"
using namespace cv_functions;

class CVTools {
public:
    explicit CVTools(std::shared_ptr<cv_tools::msg::VisionMsg> msg) : msg(msg) {}

    // 红圆检测
    cv::Mat red_circle_detect(const cv::Mat& frame) {
        std::vector<cv::Vec3f> circles; // 存储霍夫圆
        cv::Mat frame_copy = frame.clone();
        cvPipeline<cv::Mat> red_circle_pipe;
        red_circle_pipe.did(GaussianBlur, 3)
                       .did(cvtColor, cv::COLOR_BGR2HSV)
                       .did(double_mask, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), 
                                         cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255))
                        //dp=2, minDist=50, param1=50, param2=40, minRadius=20, maxRadius=0
                       .did(HoughCircles, std::ref(circles), cv::HOUGH_GRADIENT, 2, 50, 50, 40, 20, 0)
                       .process(frame_copy);

        for (const auto& circle : circles) {
            mark_circle(frame_copy, circle, "red circle", 0, 0);
            //填充ros消息
            fill_circle_msg(true, circle[1] - frame_copy.rows/2, circle[0] - frame_copy.cols/2);
            return frame_copy;
        }
        fill_circle_msg(false, 0, 0);
        return frame_copy;
    }

    // 黄方检测
    cv::Mat yellow_square_detect(const cv::Mat& frame) {
        std::vector<std::vector<cv::Point>> contours; // 存储轮廓
        cv::Mat thresh, edges;
        cv::Mat frame_copy = frame.clone();

        cvPipeline<cv::Mat> image_pipe;
        image_pipe.did(cvtColor, cv::COLOR_BGR2HSV) // 图像管道
                  .did(mask, cv::Scalar(20, 100, 100), cv::Scalar(40, 255, 255))
                  .did(cvtColor, cv::COLOR_BGR2GRAY)
                  .did(threshold, std::ref(thresh), 150, 255, cv::THRESH_BINARY)
                  .did(Canny, std::ref(edges), 100, 200)
                  .did(findContours, std::ref(contours), cv::RETR_TREE, cv::CHAIN_APPROX_NONE)
                  .process(frame_copy);
        
        cvPipeline<std::vector<std::vector<cv::Point>>> contours_pipe; // 轮廓管道
        contours_pipe.did(filter_contours_by_area, 500)
                     .did(filter_contours_by_aspect_ratio, 0.8, 1.25)
                     .did(filter_contours_by_centroid, 20)
                     .process(contours);

        for (auto& contour : contours) {
            std::vector<cv::Point> approx; // 存储近似多边形
            cvPipeline<std::vector<cv::Point>> approx_pipe;
            approx_pipe.did(approxPolyDP, approx, 0.03 * cv::arcLength(contour, true), true)
                       .process(contour);
            if (approx.size() == 4) {
                mark_contour(frame_copy, contour, "yellow square", 0, 0);
                // 填充ros消息
                cv::Moments m = cv::moments(contour);
                fill_square_msg(true, static_cast<int>(m.m01 / m.m00) - frame_copy.rows / 2,
                                            static_cast<int>(m.m10 / m.m00) - frame_copy.cols / 2);
                return frame_copy;
            }
        }
        fill_square_msg(false, 0, 0);
        return frame_copy;
    }

    // hsv空间的霍夫检测
    cv::Mat hsv_detect(cv::Mat& frame) {
        cv::Mat frame_copy = frame.clone();

        std::map<std::string, std::pair<cv::Scalar, cv::Scalar>> hsv_colors = {
            {"blue", {cv::Scalar(90, 50, 50), cv::Scalar(130, 255, 255)}},
            {"green", {cv::Scalar(40, 50, 50), cv::Scalar(80, 255, 255)}},
            {"red1", {cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255)}},
            {"red2", {cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255)}},
            {"yellow", {cv::Scalar(20, 100, 100), cv::Scalar(40, 255, 255)}}
        };

        for (auto& [color_name, range] : hsv_colors) {
            std::vector<std::vector<cv::Point>> contours; // 存储轮廓
            cv::Mat thresh, edges; // 存储边缘
            
            cvPipeline<cv::Mat> image_pipe;
            image_pipe.did(cvtColor, cv::COLOR_BGR2HSV) // 图像管道
                      .did(mask, cv::Scalar(20, 100, 100), cv::Scalar(40, 255, 255))
                      .did(cvtColor, cv::COLOR_BGR2GRAY)
                      .did(threshold, std::ref(thresh), 150, 255, cv::THRESH_BINARY)
                      .did(Canny, std::ref(edges), 100, 200)
                      .did(findContours, std::ref(contours), cv::RETR_TREE, cv::CHAIN_APPROX_NONE)
                      .process(frame);
            
            // 霍夫圆
            std::vector<cv::Vec3f> circles;
            // dp=1, minDist=50, param1=10, param2=33, minRadius=20, maxRadius=0
            cv::HoughCircles(edges, circles, cv::HOUGH_GRADIENT, 1, 50, 10, 33, 20, 0);
            for (const auto& circle : circles) {
                mark_circle(frame_copy, circle, color_name + " circle", 0, 0);
                // 填充ros消息
                fill_circle_msg(true, circle[1] - frame_copy.rows/2, circle[0] - frame_copy.cols/2);
                break;
            }

            cvPipeline<std::vector<std::vector<cv::Point>>> contours_pipe; // 轮廓管道
            contours_pipe.did(filter_contours_by_area, 500)
                         .did(filter_contours_by_aspect_ratio, 0.8, 1.25)
                         .did(filter_contours_by_centroid, 20)
                         .process(contours);
    
            // 多边形拟合
            for(auto& cnt : contours) {
                std::vector<cv::Point> approx;
                cv::approxPolyDP(cnt, approx, 0.03 * cv::arcLength(cnt, true), true);
                if (approx.size() == 4) {
                    mark_contour(frame_copy, cnt, color_name+"square", 0, 0);
                    // 填充ros消息
                    cv::Moments m = cv::moments(cnt);
                    fill_square_msg(true, static_cast<int>(m.m01 / m.m00) - frame_copy.rows / 2,
                                                static_cast<int>(m.m10 / m.m00) - frame_copy.cols / 2);
                    break;
                }
            }   
        }
        fill_circle_msg(false, 0, 0);
        fill_square_msg(false, 0, 0);
        return frame_copy;
    }

    // 霍夫直线检测
    cv::Mat line_detect(const cv::Mat& frame) {
        std::vector<cv::Vec4i> lines; // 存储直线
        cv::Mat thresh, edges;
        cv::Mat frame_copy = frame.clone();
        cvPipeline<cv::Mat> image_pipe;
        image_pipe.did(ratio_cut, 1/6, 1/6, 0, 0) // 左右各切割1/6
                  .did(GaussianBlur, 3)
                  .did(cvtColor, cv::COLOR_BGR2GRAY)
                  .did(threshold, std::ref(thresh), 100, 255, cv::THRESH_BINARY)
                  .did(Canny, std::ref(edges), 50, 200)
                  //rho=1, theta=CV_PI/180, threshold=50, minLineLength=100, maxLineGap=50
                  .did(HoughLinesP, std::ref(lines), 1, CV_PI / 180, 50, 100, 50)
                  .process(frame_copy);
        
        // 最长的直线
        int lateral_error = 0; cv::Vec4i line; auto longest_line = getLongestLine(lines);
        if (longest_line.has_value()) {
            line = longest_line.value();
            mark_line(frame_copy, line, "line", 0, 0);
            lateral_error = (line[0] + line[2]) / 2 - frame.cols / 2;
        } else {
            fill_line_msg(false, 0, 0);
            return frame_copy;
        }

        // 控制器优化
        if (std::abs(lateral_error) < 10) { lateral_error = 0;
        } else { lateral_error *= 1;
        }

        double line_angle = std::atan2(line[3] - line[1], line[2] - line[0]);
        double line_angle_body = line_angle - CV_PI / 2;
        line_angle_body = std::fmod(line_angle_body, 2 * CV_PI);
        double desired_angle = 0;
        double angle_error = line_angle - desired_angle;
        if (angle_error > CV_PI) { angle_error -= 2 * CV_PI; }

        angle_error = angle_error / (CV_PI / 2);
        if (std::abs(angle_error) < 0.1) {
            angle_error = 0;
        } else {
            angle_error = std::tanh(angle_error) * 0.3;
        }
        
        fill_line_msg(true, lateral_error, angle_error);
        return frame_copy;
    }

# pragma region private
private:
    std::shared_ptr<cv_tools::msg::VisionMsg> msg;

    void fill_circle_msg(bool is_detected, int center_x_error, int center_y_error) {
        msg->is_circle_detected = is_detected;
        msg->center_x2_error = center_x_error;
        msg->center_y2_error = center_y_error;
    }

    void fill_square_msg(bool is_detected, int center_x_error, int center_y_error) {
        msg->is_square_detected = is_detected;
        msg->center_x1_error = center_x_error;
        msg->center_y1_error = center_y_error;
    }

    void fill_line_msg(bool is_detected, int lateral_error, double angle_error) {
        msg->is_line_detected = is_detected;
        msg->lateral_error = lateral_error;
        msg->angle_error = angle_error;
    }
# pragma endregion
};
#endif
