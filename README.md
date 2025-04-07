# Quadcopter(简称Quad库)  
(个人整合)一个飞控有关的repositories，致力于打造最好的库(谦源广道 2025.3.20) 已关联内容ros2/mavros/opencv/pcl/cpp/py/rust?  

## core/core.rs  
cpp以及对应rust实现，包含飞控任务节点和飞行控制节点，外加target、velocity、path等附加实现，方便直接调用  
  
## cv_tools/vision_py  
py以及对应cpp实现了，包含一个视觉节点，接受ros2_tools的cv_bridge消息，进行视觉处理与发布。  
cpp额外实现了管道写法，通过泛型函数+泛型类型，对传统opencv的处理流程进行重构，减小拷贝开销，包扩部分opencv2.hpp的库函数重构。  
  
## ros2_tools  
cpp实现的几个功能性节点，包括lidar转发，图像camera转发，里程计消息的px4桥接，d435等的转发。
