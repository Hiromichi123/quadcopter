#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cmath>

#include "cv_pipeline.hpp"

class CVTools {
public:
    explicit CVTools(rclcpp::Node::SharedPtr node) : node(node) {}

// 基础静态
# pragma region base_static_functions
    // 初始化 VideoWriter
    static cv::VideoWriter init_video_writer(cv::VideoCapture& capture, std::string& output_filename = "output.mp4") {
        return cv::VideoWriter(output_filename, 
                            cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 
                            capture.get(cv::CAP_PROP_FPS), 
                            cv::Size(static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH)),
                                    static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT)),));
    }

    // 保存视频帧
    static void save(const cv::Mat& frame, cv::VideoWriter& writer) {
        writer.write(frame);
    }

    // 标记坐标
    static cv::Mat mark(const std::vector<cv::Point>& contour, cv::Mat& frame_copy, int originX, int originY) {
        cv::Rect rect = cv::boundingRect(contour);
        cv::rectangle(frame_copy, cv::Point(originX + rect.x - 5, originY + rect.y - 5),
                      cv::Point(originX + rect.x + rect.width + 5, originY + rect.y + rect.height + 5),
                      cv::Scalar(0, 255, 0), 2);

        cv::Moments M = cv::moments(contour);
        int center_x = static_cast<int>(M.m10 / M.m00);
        int center_y = static_cast<int>(M.m01 / M.m00);
        cv::circle(frame_copy, cv::Point(originX + center_x, originY + center_y), 1, cv::Scalar(255, 0, 255), 1);
        cv::putText(frame_copy, "[" + std::to_string(originX + center_x) + "," + std::to_string(originY + center_y) + "]",
                    cv::Point(originX + center_x, originY + center_y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);

        return frame_copy;
    }

    // 逆光补偿（不好用）
    static cv::Mat backlight_compensation(const cv::Mat& frame) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
        cv::Mat lab;
        cv::cvtColor(frame, lab, cv::COLOR_BGR2LAB);
        std::vector<cv::Mat> lab_channels;
        cv::split(lab, lab_channels);
        clahe->apply(lab_channels[0], lab_channels[0]);
        cv::merge(lab_channels, lab);
        cv::Mat result;
        cv::cvtColor(lab, result, cv::COLOR_LAB2BGR);
        return result;
    }

    // 显示视频帧
    void show(const cv::Mat& image, const std::string& window_name = "frame") {
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);
        cv::imshow(window_name, image);
    }
# pragma endregion

// 任务驱动
# pragma region task_functions
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

        if (!circles.empty()) {
            for (const auto& circle : circles) {
                cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
                int radius = cvRound(circle[2]);
                cv::circle(frame_copy, center, radius, cv::Scalar(0, 255, 0), 2);
                node->msg.is_circle_detected = true;
                node->msg.center_x2_error = center.y - frame_copy.rows / 2;
                node->msg.center_y2_error = center.x - frame_copy.cols / 2;
                return frame_copy;
            }
        }

        node->msg.is_circle_detected = false;
        node->msg.center_x2_error = 0;
        node->msg.center_y2_error = 0;
        return frame_copy;
    }

    // 黄方检测
    cv::Mat yellow_square_detect(const cv::Mat& frame) {
        std::vector<std::vector<cv::Point>> contours; // 存储轮廓
        cv::Mat frame_copy = frame.clone();

        cv_pipeline image_pipe;
        image_pipe.do(cv::COLOR_BGR2HSV) // 图像管道
                  .do(mask, cv::Scalar(20, 100, 100), cv::Scalar(40, 255, 255))
                  .do(cv::COLOR_BGR2GRAY)
                  .do(Canny, 100, 200)
                  .do(threshold, 150, 255, cv::THRESH_BINARY)
                  .do(findContours, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE)
                  .process(&frame_copy);
        
        cv_pipeline contours_pipe; // 轮廓管道
        contours_pipe.do(filter_contours_by_area, 500)
                     .do(filter_contours_by_aspect_ratio, 0.8, 1.25)
                     .do(filter_contours_by_centroid, 20)
                     .process(&contours)

        for (const auto& contour : contours) {
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contour, approx, 0.03 * cv::arcLength(contour, true), true);
            if (approx.size() == 4) {
                cv::Moments M = cv::moments(contour);
                int center_x = static_cast<int>(M.m10 / M.m00);
                int center_y = static_cast<int>(M.m01 / M.m00);
                cv::putText(frame_copy, "2", cv::Point(center_x - 40, center_y - 40),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
                frame_copy = mark(contour, frame_copy, 0, 0);
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

    // 过滤轮廓，并执行检测
    cv::Mat detect_contours(const std::vector<std::vector<cv::Point>>& contours, const cv::Mat& frame) {
        cv::Mat frame_copy = frame.clone();
        for (const auto& contour : contours) {
            cv::Rect rect = cv::boundingRect(contour);
            if (0.8 < static_cast<double>(rect.width) / rect.height && static_cast<double>(rect.width) / rect.height < 1.25) {
                cv::Mat frame_roi = frame(cv::Range(rect.y - 5, rect.y + rect.height + 5),
                                      cv::Range(rect.x - 5, rect.x + rect.width + 5)).clone();
                if (!frame_roi.empty()) {
                    hsv_detect(frame_copy, frame_roi, contour);
                }
            }
        }
        return frame_copy;
    }

    // hsv空间的霍夫检测
    void hsv_detect(cv::Mat& frame_copy, const cv::Mat& roi_img, const std::vector<cv::Point>& contour) {
        cv::Rect rect = cv::boundingRect(contour);

        std::map<std::string, std::pair<cv::Scalar, cv::Scalar>> hsv_colors = {
            {"blue", {cv::Scalar(90, 50, 50), cv::Scalar(130, 255, 255)}},
            {"green", {cv::Scalar(40, 50, 50), cv::Scalar(80, 255, 255)}},
            {"red1", {cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255)}},
            {"red2", {cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255)}},
            {"yellow", {cv::Scalar(20, 100, 100), cv::Scalar(40, 255, 255)}}
        };

        cv::Mat hsv_img;
        cv::cvtColor(roi_img, hsv_img, cv::COLOR_BGR2HSV);

        for (const auto& [color_name, range] : hsv_colors) {
            cv::Mat hsv_mask;
            cv::inRange(hsv_img, range.first, range.second, hsv_mask);
            cv::Mat hsv_result;
            cv::bitwise_and(roi_img, roi_img, hsv_result, hsv_mask);

            if (cv::countNonZero(hsv_mask) < 1000) {
                continue;
            }

            cv::Mat hsv_gray;
            cv::cvtColor(hsv_result, hsv_gray, cv::COLOR_BGR2GRAY);
            cv::Mat hsv_edges;
            cv::Canny(hsv_gray, hsv_edges, 100, 200);
            cv::Mat hsv_thresh;
            cv::threshold(hsv_gray, hsv_thresh, 150, 255, cv::THRESH_BINARY);

            std::vector<std::vector<cv::Point>> hsv_contours;
            cv::findContours(hsv_thresh, hsv_contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

            std::vector<std::vector<cv::Point>> valid_contours;
            for (const auto& cnt : hsv_contours) {
                if (cv::contourArea(cnt) > 500) {
                    valid_contours.push_back(cnt);
                }
            }

            std::vector<std::vector<cv::Point>> possible_contours = filter_contours_by_centroid(valid_contours, 20);

            for (const auto& cnt : possible_contours) {
                if (color_name == "red1" || color_name == "red2") {
                    std::vector<cv::Vec3f> circles;
                    cv::HoughCircles(hsv_edges, circles, cv::HOUGH_GRADIENT, 1, 50, 10, 33, 20, 0);
                    if (!circles.empty()) {
                        for (const auto& circle : circles) {
                            cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
                            int radius = cvRound(circle[2]);
                            cv::putText(frame_copy, "1", cv::Point(rect.x - 5 + center.x - 40, rect.y - 5 + center.y - 40),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
                            frame_copy = mark(cnt, frame_copy, rect.x, rect.y);
                            node->msg.is_circle_detected = true;
                            node->msg.center_x2_error = rect.y - 5 + center.y - frame_copy.rows / 2;
                            node->msg.center_y2_error = rect.x - 5 + center.x - frame_copy.cols / 2;
                        }
                    }
                }

                std::vector<cv::Point> approx;
                cv::approxPolyDP(cnt, approx, 0.03 * cv::arcLength(cnt, true), true);
                if (approx.size() == 4) {
                    cv::Moments M = cv::moments(cnt);
                    int center_x = static_cast<int>(M.m10 / M.m00);
                    int center_y = static_cast<int>(M.m01 / M.m00);
                    if (color_name == "yellow") {
                        cv::putText(frame_copy, "2", cv::Point(rect.x + center_x - 40, rect.y + center_y - 40),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
                        frame_copy = mark(cnt, frame_copy, rect.x, rect.y);
                        node->msg.is_square_detected = true;
                        node->msg.center_x1_error = rect.y - 5 + center_y - frame_copy.rows / 2;
                        node->msg.center_y1_error = rect.x - 5 + center_x - frame_copy.cols / 2;
                    }
                }
            }
        }
    }

    // 霍夫直线，TODO：卡尔曼滤波
    cv::Mat line_detect(const cv::Mat& frame) {
        cv::Mat frame_copy = frame.clone();
        cv::Mat edges;
        cv::Canny(frame, edges, 50, 200, 3);

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 100, 50);

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
        if (std::abs(lateral_error) < 10) {
            lateral_error = 0;
        } else {
            lateral_error *= 1;
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
# pragma endregion

private:
    rclcpp::Node::SharedPtr node;
};