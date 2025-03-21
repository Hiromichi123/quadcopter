#include <opencv2/opencv.hpp>

// 图像处理函数
#pragma region image
// 均值滤波
void GaussianBlur(cv::Mat& image, int kernelSize) {
    cv::GaussianBlur(*image, *image, cv::Size(kernelSize, kernelSize), 0);
}

// 转换颜色空间
// code: cv::COLOR_BGR2GRAY, cv::COLOR_BGR2HSV, cv::COLOR_BGR2RGB, cv::COLOR_BGR2XYZ, cv::COLOR_BGR2YCrCb
void cvtColor(cv::Mat& image, int code) {
    cv::cvtColor(*image, *image, code);
}

// 二值化
// type: cv::THRESH_BINARY, cv::THRESH_BINARY_INV, cv::THRESH_TRUNC, cv::THRESH_TOZERO, cv::THRESH_TOZERO_INV
void threshold(cv::Mat& image, double thresh, double maxval, int type) {
    cv::threshold(*image, *image, thresh, maxval, type);
}

// Canny边缘检测
void Canny(cv::Mat& image, double threshold1, double threshold2) {
    cv::Canny(*image, *image, threshold1, threshold2);
}

// 提取轮廓
// mode: cv::RETR_EXTERNAL, cv::RETR_LIST, cv::RETR_CCOMP, cv::RETR_TREE
void findContours(cv::Mat& image, std::vector<std::vector<cv::Point>>& contours, int mode, int method) {
    cv::findContours(*image, contours, mode, method);
}

// 单区间掩膜操作
void mask(cv::Mat& image, const cv::Scalar& lower, const cv::Scalar& upper) {
    cv::Mat mask;
    cv::inRange(*image, lower, upper, mask);
    cv::bitwise_and(*image, *image, *image, mask); // 与操作
}

// 双区间掩膜操作
void mask(cv::Mat& image1, cv::Scalar& lower1, cv::Scalar& upper1, cv::Scalar& lower2, cv::Scalar& upper2) {
    cv::Mat mask1, mask2;
    cv::inRange(*image1, lower1, upper1, mask1);
    cv::inRange(*image1, lower2, upper2, mask2);
    cv::bitwise_or(mask1, mask2, mask1); // 或操作
    cv::bitwise_and(*image1, *image1, *image1, mask1); // 与操作
}

// 霍夫圆检测
void HoughCircles(cv::Mat& image, std::vector<cv::Vec3f>& circles, int method, double dp, double minDist, double param1, double param2, int minRadius, int maxRadius) {
    cv::HoughCircles(*image, circles, method, dp, minDist, param1, param2, minRadius, maxRadius);
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