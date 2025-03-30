#ifndef CV_PIPELINE_HPP
#define CV_PIPELINE_HPP
#include <functional>
#include <vector>
#include "cv_functions.hpp"

namespace cv_functions {
template<typename T>
class cvPipeline {
public:
    // 添加处理
    template<typename Func, typename... Args>
    cvPipeline& did(Func func, Args... args) {
        steps.push_back([func, args...](T& data) { // Lambda 封装
            func(data, args...);
        });
        return *this; // 支持链式
    }

    // 最后调用，处理泛型
    void process(T& data) {
        for (auto& step : steps) { // 迭代执行函数
            step(data);
        }
    }

private:
    std::vector<std::function<void(T&)>> steps; // 函数容器
};
} // namespace cv_functions
#endif
