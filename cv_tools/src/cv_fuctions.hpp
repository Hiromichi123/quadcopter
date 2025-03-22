#include <opencv2/opencv.hpp>

// 图像处理函数
#pragma region image
// 均值滤波
void GaussianBlur(cv::Mat& image, int kernelSize) {
    cv::GaussianBlur(image, image, cv::Size(kernelSize, kernelSize), 0);
}

// 转换颜色空间
// code: cv::COLOR_BGR2GRAY, cv::COLOR_BGR2HSV, cv::COLOR_BGR2RGB, cv::COLOR_BGR2XYZ, cv::COLOR_BGR2YCrCb
void cvtColor(cv::Mat& image, int code) {
    cv::cvtColor(image, image, code);
}

// 二值化，返回threshold
// type: cv::THRESH_BINARY, cv::THRESH_BINARY_INV, cv::THRESH_TRUNC, cv::THRESH_TOZERO, cv::THRESH_TOZERO_INV
void threshold(cv::Mat& image, double threshold, double maxval, int type) {
    cv::threshold(image, threshold, maxval, type);
}

// Canny边缘检测，返回edges
void Canny(cv::Mat& image, cv::Mat& edges, double threshold1, double threshold2) {
    cv::Canny(image, edges, threshold1, threshold2);
}

// 提取轮廓，返回contours
// mode: cv::RETR_EXTERNAL, cv::RETR_LIST, cv::RETR_CCOMP, cv::RETR_TREE
void findContours(cv::Mat& image, std::vector<std::vector<cv::Point>>& contours, int mode, int method) {
    cv::findContours(image, contours, mode, method);
}

// 单区间掩膜操作
void mask(cv::Mat& image, const cv::Scalar& lower, const cv::Scalar& upper) {
    cv::Mat mask;
    cv::inRange(image, lower, upper, mask);
    cv::bitwise_and(image, image, image, mask); // 与操作
}

// 双区间掩膜操作
void mask(cv::Mat& image, cv::Scalar& lower1, cv::Scalar& upper1, cv::Scalar& lower2, cv::Scalar& upper2) {
    cv::Mat mask1, mask2;
    cv::inRange(image, lower1, upper1, mask1);
    cv::inRange(image, lower2, upper2, mask2);
    cv::bitwise_or(mask1, mask2, mask1); // 或操作
    cv::bitwise_and(image, image, image, mask1); // 与操作
}

// 霍夫圆检测
void HoughCircles(cv::Mat& image, std::vector<cv::Vec3f>& circles, int method, double dp, double minDist, double param1, double param2, int minRadius, int maxRadius) {
    cv::HoughCircles(image, circles, method, dp, minDist, param1, param2, minRadius, maxRadius);
}

// 霍夫直线检测
void HoughLinesP(cv::Mat& image, std::vector<cv::Vec4i>& lines, double rho, double theta, int threshold, double minLineLength, double maxLineGap) {
    cv::HoughLinesP(image, lines, rho, theta, threshold, minLineLength, maxLineGap);
}

// 逆光补偿（不好用）
cv::Mat backlight_compensation(const cv::Mat& frame) {
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
#pragma endregion

// 多轮廓处理函数
#pragma region contours
// 过滤靠近轮廓
static std::vector<std::vector<cv::Point>> filter_contours_by_centroid(const std::vector<std::vector<cv::Point>>& contours, double min_dist = 20) {
    std::vector<cv::Point2f> contour_centers;
    std::vector<std::vector<cv::Point>> valid_contours;
    for (const auto& cnt : contours) {
        cv::Moments M = cv::moments(cnt);
        if (M.m00 != 0) {
            float cx = static_cast<float>(M.m10 / M.m00);
            float cy = static_cast<float>(M.m01 / M.m00);

            bool is_far = true;
            for (const auto& center : contour_centers) {
                if (cv::norm(cv::Point2f(cx, cy) - center) < min_dist) {
                    is_far = false;
                    break;
                }
            }

            if (is_far) {
                contour_centers.emplace_back(cx, cy);
                valid_contours.push_back(cnt);
            }
        }
    }
    return valid_contours;
}

// 过滤小轮廓
static std::vector<std::vector<cv::Point>> filter_contours_by_area(const std::vector<std::vector<cv::Point>>& contours, double minArea) {
    std::vector<std::vector<cv::Point>> valid_contours;
    std::copy_if(contours.begin(), contours.end(), std::back_inserter(valid_contours),
                    [minArea](const std::vector<cv::Point>& cnt) {
                        return cv::contourArea(cnt) > minArea;
                    });
    return valid_contours;
}

// 过滤长宽比例轮廓
static std::vector<std::vector<cv::Point>> filter_contours_by_aspect_ratio(const std::vector<std::vector<cv::Point>>& contours, double minRatio, double maxRatio) {
    std::vector<std::vector<cv::Point>> valid_contours;
    std::copy_if(contours.begin(), contours.end(), std::back_inserter(valid_contours),
                    [minRatio, maxRatio](const std::vector<cv::Point>& cnt) {
                        cv::Rect bbox = cv::boundingRect(cnt);
                        double aspectRatio = static_cast<double>(bbox.width) / bbox.height;
                        return (aspectRatio >= minRatio && aspectRatio <= maxRatio);
                    });
    return valid_contours;
}
#pragma endregion

// 单轮廓处理函数
#pragma region contour
// 轮廓近似
void approxPolyDP(const std::vector<cv::Point>& contour, std::vector<cv::Point>& approx, double epsilon, bool closed) {
    cv::approxPolyDP(contour, approx, epsilon, closed);
}
#pragma endregion

// 基础函数
#pragma region base
// 初始化VideoWriter
cv::VideoWriter init_video_writer(cv::VideoCapture& capture, std::string& output_filename = "output.mp4") {
    return cv::VideoWriter(output_filename, 
                        cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 
                        capture.get(cv::CAP_PROP_FPS), 
                        cv::Size(static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH)),
                                static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT)),));
}

// 保存视频帧
void save(cv::Mat& frame, cv::VideoWriter& writer) {
    writer.write(frame);
}

// 显示视频帧
void show(cv::Mat& image, std::string& window_name = "frame") {
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::imshow(window_name, image);
}

// 按比例切割图像边缘
cv::Mat ratio_cut(const cv::Mat& frame, float left_ratio=0, float right_ratio=0, float top_ratio=0, float bottom_ratio=0) {
    int left = frame.cols * left_ratio;
    int right = frame.cols - frame.cols * right_ratio;
    int top = frame.rows * top_ratio;
    int bottom = frame.rows - frame.rows * bottom_ratio;
    return frame(cv::Range(top, bottom), cv::Range(left, right));
}
#pragma endregion

// 绘制函数
#pragma region draw
// 标记轮廓+中心+文本+坐标（多边形等）
cv::Mat mark_contour(cv::Mat& frame_copy, std::vector<cv::Point>& contour, std::string& text, int originX, int originY) {
    cv::Rect rect = cv::boundingRect(contour);
    cv::rectangle(frame_copy, cv::Point(originX + rect.x - 5, originY + rect.y - 5),
                    cv::Point(originX + rect.x + rect.width + 5, originY + rect.y + rect.height + 5),
                    cv::Scalar(0, 255, 0), 2); // 绘制外接矩形框
    cv::Moments M = cv::moments(contour);
    int center_x = static_cast<int>(M.m10 / M.m00);
    int center_y = static_cast<int>(M.m01 / M.m00);
    cv::circle(frame_copy, cv::Point(originX + center_x, originY + center_y), 1, cv::Scalar(255, 0, 255), 2); // 中心
    cv::putText(frame_copy, text + "[" + std::to_string(originX + center_x) + "," + std::to_string(originY + center_y) + "]",
                cv::Point(originX + center_x, originY + center_y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1); // 文本+坐标
    return frame_copy;
}

// 标记圆+中心+文本+坐标（霍夫圆）
cv::Mat mark_circle(cv::Mat& frame_copy, cv::Vec3f& circle, std::string& text, int originX, int originY) {
    cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
    int radius = cvRound(circle[2]);
    cv::circle(frame_copy, cv::Point(originX + center.x, originY + center.y), 2, cv::Scalar(0, 255, 0), 2); // 中心
    cv::circle(frame_copy, cv::Point(originX + center.x, originY + center.y), radius, cv::Scalar(0, 255, 0), 2); // 绘制圆
    cv::putText(frame_copy, text, cv::Point(originX + center.x - 40, originY + center.y - 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
    return frame_copy;
}

// 标记直线+文本+坐标（霍夫直线）
cv::Mat mark_line(cv::Mat& frame_copy, cv::Vec4i& line, std::string& text, int originX, int originY) {
    cv::line(frame_copy, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 2); // 绘制直线
    cv::putText(frame_copy, text, cv::Point(originX + line[0], originY + line[1] - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
    return frame_copy;
}
#pragma endregion