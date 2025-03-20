#include <opencv2/opencv.hpp>
#include <functional>
#include <vector>
#include "cv_functions.hpp"

class cvPipeline {
public:
    template<typename Func, typename... Args>
    cvPipeline& do(Func func, Args... args) {
        steps.push_back([func, args...](cv::Mat& img) { // Lambda 封装带参数的函数
            func(img, args...);
        });
        return *this; // 支持链式
    }

    // 处理图像
    void process(cv::Mat& image) {
        for (auto& step : steps) { // 迭代执行函数
            step(image);
        }
    }

private:
    std::vector<std::function<void(cv::Mat&)>> steps; // 函数容器
};
