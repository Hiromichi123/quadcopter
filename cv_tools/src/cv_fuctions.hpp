#include <opencv2/opencv.hpp>

// 均值滤波
void GaussianBlur(cv::Mat& image, int kernelSize) {
    cv::GaussianBlur(*image, *image, cv::Size(kernelSize, kernelSize), 0);
}

// 转换颜色空间
// code: cv::COLOR_BGR2GRAY, cv::COLOR_BGR2HSV, cv::COLOR_BGR2RGB, cv::COLOR_BGR2XYZ, cv::COLOR_BGR2YCrCb
void cvtColor(cv::Mat& image, int code) {
    cv::cvtColor(*image, *image, code);
}

// 阈值化
// type: cv::THRESH_BINARY, cv::THRESH_BINARY_INV, cv::THRESH_TRUNC, cv::THRESH_TOZERO, cv::THRESH_TOZERO_INV
void threshold(cv::Mat& image, double thresh, double maxval, int type) {
    cv::threshold(*image, *image, thresh, maxval, type);
}

// Canny边缘检测
void Canny(cv::Mat& image, double threshold1, double threshold2) {
    cv::Canny(*image, *image, threshold1, threshold2);
}

// 显示视频帧
void show(const cv::Mat& frame, const std::string& window_name = "frame") {
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::imshow(window_name, frame);
}