关于Colcon build
1.	常规安装
o	colcon会将构建的文件（例如编译后的库、可执行文件、Python 脚本等）拷贝到 install 目录中，每次重新构建时重新复制。
o	好处是可以独立于源码目录使用和分发安装的文件。
2.	符号链接安装（使用 --symlink-install后缀）：
o	会在 install 目录中创建符号链接，这些链接指向源代码目录中的构建文件，无需重新构建或重新复制文件。
何时使用 --symlink-install
开发避免每次修改后都重新安装，节省时间。调试：如果你在调试某个包，或者希望查看源代码与安装目录之间的差异，符号链接提供了一种更便捷的方式。

rm -r /build /install /log 删除所有安装后构建的中间文件
--packages-select 指定编译某个包
ros2 pkg list | grep mavros 查找指定的包
ros2 interface show mavros_msgs/srv/CommandBool 查看某个消息、服务类型（是否存在）

_pycache_ 目录
Python 在执行时自动生成的一个目录，存放已编译的字节码文件（.pyc 文件）当运行时解释器会将源代码（.py 文件）编译成字节码并存储为 .pyc 文件，以提高下次运行的速度。假编译
•	字节码 是 Python 代码的中间表示形式，它比源代码（.py 文件）更接近机器代码，但仍然保持与 Python 解释器平台无关。字节码文件可以被 Python 解释器直接执行。

entry_points
在 setup.py 中的 entry_points 配置项用于定义 ROS 2 Python 包中的 可执行命令，这些命令可以直接在命令行中运行。它们通常用于将包中的函数暴露为命令行工具。
entry_points={
    'console_scripts': [
        'ball_calibration = vision.ball_calibration:main',
        'ball_depth = vision.ball_depth:main',
        'd_ballposition = vision.d_ballposition:main'
    ],
},
•	entry_points：一个字典，指定包的入口点。通过设置入口点，Python 包可以将其功能暴露给外部程序（如命令行工具），使其能够通过命令行执行特定的函数或脚本。
•	'console_scripts'：这是 entry_points 的一个特定部分，专门用于定义在命令行中可以直接执行的脚本。
o	这些命令将会在系统的 PATH 中创建一个可执行的命令，可以直接从终端调用。
o	这些命令会映射到包中的 Python 函数，通常是某个模块的 main 函数。

PCL点云库索引
Features 特征计算
Filtering 滤波/降采样
I/O 输入输出
KeyPoints 关键点获取
KdTree、Octree
Recognotion 识别
Registration 配准
Sample Consensus 随机抽样一致性
Segmentation 分割
Surface 表面处理
Visualization 可视化 

CMakeLists要点
1. PROGRAMS 关键字
install(PROGRAMS
    scripts/ballposition.py
  DESTINATION lib/${PROJECT_NAME}
)
安装脚本文件（通常是Python脚本或可执行Shell脚本）。
o	PROGRAMS 会直接将这些文件复制到指定的目标路径（这里是 lib/${PROJECT_NAME}）。
o	这些文件需要在源代码目录中已有执行权限。
________________________________________
2. TARGETS 关键字
install(TARGETS
livox_to_pointcloud2 point_detect_node 
  DESTINATION lib/${PROJECT_NAME}
)
安装已编译的目标文件（如C++程序生成的二进制文件）。
o	TARGETS 用于指定通过CMake生成的目标（如通过 add_executable() 或 add_library() 定义的目标）。
o	安装时，CMake会自动找到这些目标文件的正确位置并复制到指定的安装目录。

关于Livox_SDK2 & livox_ros_driver2 & Fastlio2
安装livox_ros_driver时注意:
在ubuntu22.04的环境下编译，需要在/Livox-SDK/sdk_core/src/base文件下的thread_base.h中添加#include <memory>（github issue里很深的地方）

关于ROS2的改动
注意是geometry_msgs::msg::………，容易与geometry::msg::………..混淆
mavros_msgs也改为了mavros_msgs::msg::或者mavros_msgs::srv::
包含头文件由大写驼峰改为小写下划线

ROS2提供的别名
简化写法rclcpp::TimerBase::SharedPtr pre_fly_timer;
原写法std::shared_ptr<rclcpp::TimerBase> pre_fly_timer; 
同样适用于node、subscriber、publisher等，免去类型声明

循环注意事项
一切ros2节点使用的包括起飞前预检查，持续时间预期较长（一般以5s为界限）或有概率无法达成跳出循环，循环条件一律添加rolcpp/pclpy::ok()保证ros正常退出。

msg的名称问题
逆天的命名规范
在消息文件内部bool isShapeDetecedt此类驼峰命名法禁用，写成is_shape_detected，msg文件名必须使用LidarPose.msg大写驼峰，首字母必须大写，实际包含进源程序却要写做#include<~/~/~/lidar_pose>。

关于代码规范
1. 变量命名
（1）普通变量
•	驼峰命名法（camelCase）：
用于局部变量、函数参数double maxSpeed;
下划线命名法（snake_case）：推荐double max_speed;

（2）成员变量
•	前缀 m_，个人不推荐
•	尾缀 _（ai工具非常喜欢用，可以快速判断队友的代码是不是跑的）
2. 函数命名
•	小写驼峰（camelCase） 用于一般函数（STL、Qt、部分C++开源库）
•	大写驼峰（PascalCase） 用于类成员函数（Windows API、UE、部分游戏引擎）不推荐

3. 类命名
•	PascalCase（大写驼峰）
•	抽象类（接口）不推荐

4.宏定义（尽量少用宏，推荐使用 constexpr）全大写 #define MAX_PLAYERS 4
•	枚举值

5. 命名空间 全小写（snake_case）
6. 文件命名
•	头文件（.h/.hpp）： snake_case.h推荐 或 PascalCase.hpp二选一
•	源文件（.cpp）： snake_case.cpp推荐 或 PascalCase.cpp二选一
7. 其它命名
•	迭代器使用 it 作为前缀std::vector<int>::iterator it;
•	布尔变量以 is/has/can 开头bool isShapedDeteced;
其他杂规略

8.变量或函数名前的单个下划线 _name
通常用于表示私有（private）或内部（internal）变量/函数
（1）类的私有成员
class Cat {
    privte int _a;
    private float _b;
};

9.类声明强调权限，建议没有特殊情况一律用protected，方便继承使用

ROS2的循环机制
ros2的循环有三种，
spin(节点)：等待回调队列，有消息进入尽快处理，否则阻塞
spin_once(节点)：查看处理回调一次，通常结合sleep手动调用
（注意：在rclpy的rate.sleep()会在循环中出现莫名其妙的阻塞，建议不要使用，但rclcpp正常，目前未知）
基于Timer的发布：也算一种循环，也不需要手动控制频率，由定时器决定

Rclpy的隐藏bug
如果包名和脚本重名，那么在使用ros2 run时会导致查找失败。原因是install/包名下，有lib和local/lib两个相似的路径，lib存放的是执行脚本，local/lib才是包的真正位置，python会优先查找前者找到脚本名并认为是包。
有解决办法：在main（entry_point入口）里延迟import。但建议还是在包和脚本命名时区分彻底避免问题

Gazebo的iris添加模组camera
对于model-iris来说组成的基本物理单元是link标签，比如base_link，camera_link，但要注意如果要构造物理连接，必须要写上joint标签，来表达连接的物理关系。插件plugin标签需要作为link的子模块存在。

Qos服务质量
某些通信如订阅px4的本地定位，对qos有高要求必须匹配。如：
rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // px4_local_position 订阅
        local_position_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", qos_profile, std::bind(&LidarDataNode::localPositionCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "PX4 Local Position subscription successful.");

Rclpy的安装
比起rclcpp只有install一种安装可执行文件的方式，rclpy可以选择install（结果是install里只有对应的.py脚本，只能作为py文件ros2 run，但不识别为节点），或者使用ament_python通过设置setup.py来安装为节点。两种方式有差别注意区分。
    entry_points={
        'console_scripts': [
            'vision_node = scripts.vision_node:main'  # 确保入口路径正确
        ],
    },
    data_files=[
    (os.path.join("share", package_name), ["package.xml"]), #package.xml必须要被安装到install/share/your_package/下
    (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")), # 所有scripts目录下py文件都被安装
],
注意cmakelist里也要显式安装install(FILES package.xml DESTINATION share/${PROJECT_NAME})才行
cmake写法也有讲究，比如标准的python安装语句find_package(Python3 REQUIRED COMPONENTS Interpreter)，以及ament_cmake自带的find_package(ament_cmake_python REQUIRED)都可以用于安装py相关文件

关于CPP的几个关注点
函数参数的string引用在赋默认值（传递字面值（右值））时，注意添加const修饰，要求不可修改（左值）。

语法错误：某函数传参(mask, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), 
                                 cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255))
实际参数列表(cv::Scalar& lower1, cv::Scalar& upper1, cv::Scalar& lower2, cv::Scalar& upper2)
问题：临时对象传参，无法绑定到非常量引用
解决方法还是修改为const

对于函数模版的传参，如果有重载函数编译器很可能无法推导，要谨慎使用。
在 C++ 模板推导中，当你传递一个参数（比如 cv::Mat thresh）给一个模板函数时，C++ 会自动根据参数的类型推导出模板的实际类型。默认情况下，C++ 会推导出一个 常量引用（const T&）而不是 普通引用（T&），这通常是为了保证函数内对数据的修改不影响原始数据，避免修改意外发生。
小总结：对于cpp的函数传参，左值（大型类）常用指针传参或者const&修饰，对于内置类型int等直接拷贝或者T&&（右值引用）表示可移动（cpp11后，完美转发forword）
小拓展：
完美转发：使用 T&& + std::forward<T>
如果一个模板函数的参数需要 同时支持左值和右值，可以用 通用引用（万能引用）
用于譬如string，vector

A类和B类互相持有对方指针，但需要访问对象，典型的前向声明+双向依赖+不完整定义，解决方案：1抽象类将共有部分独立出来、互相包含.h
				2中介者模式实现类之间的交互
				3依赖注入
C#可以在成员函数前添加权限，cpp必须写在规定范围。

Rust 的 ROS 2 构建体系中：
所有通过 rosidl_generator_rs（或 rosidl_typesupport_rust）生成的 .rs 消息类型，不是作为独立 crate 发布的；
它们是通过 r2r crate 统一引入的；
r2r crate 的构建过程中会根据你工作空间里的消息包，把类型绑定自动包含进去。
所以，所有的消息最终都以如下命名空间方式暴露给你：
r2r::<package_name>::msg::<MessageName>
且rust不支持生成自定义消息，rust包中仍需要camke辅助构建消息
