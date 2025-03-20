#include <opencv2/opencv.hpp>

// 霍夫直线检测
cv::Mat line_detect(cv::Mat& image) {
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(*image, lines, 1, CV_PI / 180, 50, 50, 10);
    cv::Mat hl_copy = cv::Mat::zeros(image.size(), CV_8UC3);
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        cv::line(hl_copy, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    }
    return hl_copy;
}