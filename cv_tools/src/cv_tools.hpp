#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cmath>

#include "cv_pipeline.hpp"
#include "cv_functions.hpp"

class CVTools {
public:
    explicit CVTools(rclcpp::Node::SharedPtr node) : node(node) {}

    // 红圆检测
    cv::Mat red_circle_detect(const cv::Mat& frame) {
        std::vector<cv::Vec3f> circles; // 存储霍夫圆
        cv::Mat frame_copy = frame.clone();
        cvPipeline red_circle_pipe;
        red_circle_pipe.do(GaussianBlur, 3)
                       .do(cv::COLOR_BGR2HSV)
                       .do(mask, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), 
                                 cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255))
                        //dp=2, minDist=50, param1=50, param2=40, minRadius=20, maxRadius=0
                       .do(HoughCircles, circles, cv::HOUGH_GRADIENT, 2, 50, 50, 40, 20, 0)
                       .process(&frame_copy);

        for (const auto& circle : circles) {
            frame_copy = mark_circle(frame_copy, circle, "red circle", 0, 0);
            //填充ros消息
            node->msg.is_circle_detected = true;
            node->msg.center_x2_error = center.y - frame_copy.rows / 2;
            node->msg.center_y2_error = center.x - frame_copy.cols / 2;
            return frame_copy;
        }

        node->msg.is_circle_detected = false;
        node->msg.center_x2_error = 0;
        node->msg.center_y2_error = 0;
        return frame_copy;
    }

    // 黄方检测
    cv::Mat yellow_square_detect(const cv::Mat& frame) {
        std::vector<std::vector<cv::Point>> contours; // 存储轮廓
        cv::Mat threshold, edges;
        cv::Mat frame_copy = frame.clone();

        cv_pipeline image_pipe;
        image_pipe.do(cv::COLOR_BGR2HSV) // 图像管道
                  .do(mask, cv::Scalar(20, 100, 100), cv::Scalar(40, 255, 255))
                  .do(cv::COLOR_BGR2GRAY)
                  .do(threshold, threshold, 150, 255, cv::THRESH_BINARY)
                  .do(Canny, edges, 100, 200)
                  .do(findContours, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE)
                  .process(&frame_copy);
        
        cv_pipeline contours_pipe; // 轮廓管道
        contours_pipe.do(filter_contours_by_area, 500)
                     .do(filter_contours_by_aspect_ratio, 0.8, 1.25)
                     .do(filter_contours_by_centroid, 20)
                     .process(&contours)

        for (const auto& contour : contours) {
            std::vector<cv::Point> approx; // 存储近似多边形
            cv_pipeline approx_pipe;
            approx_pipe.do(approxPolyDP, approx, 0.03 * cv::arcLength(contour, true), true)
                       .process(&contour);
            if (approx.size() == 4) {
                frame_copy = mark_contour(frame_copy, contour, "yellow square", 0, 0);
                // 填充ros消息
                node->msg.is_square_detected = true;
                node->msg.center_x1_error = center_y - frame_copy.rows / 2;
                node->msg.center_y1_error = center_x - frame_copy.cols / 2;
                return frame_copy;
            }
        }
        node->msg.is_square_detected = false;
        node->msg.center_x1_error = 0;
        node->msg.center_y1_error = 0;
        return frame_copy;
    }

    // hsv空间的霍夫检测
    cv::Mat hsv_detect(cv::Mat& frame_copy) {
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
            cv::Mat threshold, edges; // 存储边缘
            
            cv_pipeline image_pipe;
            image_pipe.do(cv::COLOR_BGR2HSV) // 图像管道
                      .do(mask, cv::Scalar(20, 100, 100), cv::Scalar(40, 255, 255))
                      .do(cv::COLOR_BGR2GRAY)
                      .do(threshold, threshold, 150, 255, cv::THRESH_BINARY)
                      .do(Canny, edges, 100, 200)
                      .do(findContours, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE)
                      .process(&frame_copy);
            
            // 霍夫圆
            std::vector<cv::Vec3f> circles;
            // dp=1, minDist=50, param1=10, param2=33, minRadius=20, maxRadius=0
            cv::HoughCircles(edges, circles, cv::HOUGH_GRADIENT, 1, 50, 10, 33, 20, 0);
            for (const auto& circle : circles) {
                frame_copy = mark_circle(frame_copy, circle, color_name + " circle", 0, 0);
                // 填充ros消息
                node->msg.is_circle_detected = true;
                node->msg.center_x2_error = rect.y - 5 + center.y - frame_copy.rows / 2;
                node->msg.center_y2_error = rect.x - 5 + center.x - frame_copy.cols / 2;
                break;
            }

            // 多边形拟合
            for(auto& cnt : contours) {
                std::vector<cv::Point> approx;
                cv::approxPolyDP(cnt, approx, 0.03 * cv::arcLength(cnt, true), true);
                if (approx.size() == 4) {
                    frame_copy = mark_contour(frame_copy, cnt, str(color_name)+"square", 0, 0);
                    // 填充ros消息
                    node->msg.is_square_detected = true;
                    node->msg.center_x1_error = rect.y - 5 + center_y - frame_copy.rows / 2;
                    node->msg.center_y1_error = rect.x - 5 + center_x - frame_copy.cols / 2;
                    break;
                }
            }   
        }
        node->msg.is_circle_detected = false;
        node->msg.center_x2_error = 0;
        node->msg.center_y2_error = 0;
        node->msg.is_square_detected = false;
        node->msg.center_x1_error = 0;
        node->msg.center_y1_error = 0;
        return frame_copy;
    }

    // 霍夫直线检测
    cv::Mat line_detect(const cv::Mat& frame) {
        std::vector<cv::Vec4i> lines; // 存储直线
        cv::Mat frame_copy = frame.clone();
        cv_pipeline image_pipe;
        image_pipe.do(ratio_cut, 1/6, 1/6) // 左右各切割1/6
                  .do(GaussianBlur, 3)
                  .do(cvtColor, cv::COLOR_BGR2GRAY)
                  .do(threshold, 100, 255, cv::THRESH_BINARY)
                  .do(Canny, 50, 200, 3)
                  //rho=1, theta=CV_PI/180, threshold=50, minLineLength=100, maxLineGap=50
                  .do(HoughLinesP, lines, 1, CV_PI / 180, 50, 100, 50);
                  .process(&frame_copy);
        
        if (lines.empty()) {
            node->msg.is_line_detected = false;
            node->msg.lateral_error = 0;
            node->msg.angle_error = 0.0;
            return frame_copy;
        }

        // 找到最长的直线
        auto best_line = *std::max_element(lines.begin(), lines.end(), [](const cv::Vec4i& a, const cv::Vec4i& b) {
            return cv::norm(cv::Point(a[0], a[1]) - cv::Point(a[2], a[3])) < cv::norm(cv::Point(b[0], b[1]) - cv::Point(b[2], b[3]));
        });
        cv::line(frame_copy, cv::Point(best_line[0], best_line[1]), cv::Point(best_line[2], best_line[3]), cv::Scalar(0, 0, 255), 3);

        int lateral_error = (best_line[0] + best_line[2]) / 2 - frame.cols / 2;

        // 控制器优化
        if (std::abs(lateral_error) < 10) { lateral_error = 0;
        } else { lateral_error *= 1;
        }

        double line_angle = std::atan2(best_line[3] - best_line[1], best_line[2] - best_line[0]);
        double line_angle_body = line_angle - CV_PI / 2;
        line_angle_body = std::fmod(line_angle_body, 2 * CV_PI);
        double desired_angle = 0;
        double angle_error = line_angle - desired_angle;
        if (angle_error > CV_PI) {
            angle_error -= 2 * CV_PI;
        }

        angle_error = angle_error / (CV_PI / 2);
        if (std::abs(angle_error) < 0.1) {
            angle_error = 0;
        } else {
            angle_error = std::tanh(angle_error) * 0.3;
        }

        node->msg.is_line_detected = true;
        node->msg.lateral_error = lateral_error;
        node->msg.angle_error = angle_error;
        return frame_copy;
    }

private:
    rclcpp::Node::SharedPtr node;
};