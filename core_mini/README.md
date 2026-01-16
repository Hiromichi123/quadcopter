# Core Mini 教学版四旋翼控制包

这是一个简化的四旋翼控制包，专门为教学目的设计，适合ROS2初学者和基础一般的同学学习。

## 包含的功能

### 基本飞行功能
- ✅ 定点飞行（fly_to_target）：飞到指定位置并悬停
- ✅ 速度控制（fly_by_velocity）：通过速度控制飞行器运动
- ✅ 起飞前检查：自动解锁和切换飞行模式

### 简化的数据类型
- **Target类**：表示目标位置（x, y, z, yaw）
- **Velocity类**：表示速度指令（vx, vy, vz, vyaw）

### 省略的功能（相比完整版core）
- ❌ PID控制器
- ❌ 持续速度飞行（带定高）
- ❌ 航点飞行（Path类）
- ❌ 复杂的get/set方法

## 文件结构

```
core_mini/
├── CMakeLists.txt              # CMake构建配置
├── package.xml                 # ROS2包描述文件
├── launch/
│   └── core_mini_launch.py    # 启动文件
└── src/
    ├── simple_types.hpp       # 目标点类、速度类的定义头
    └── quadcopter_mini.cpp    # 四旋翼控制节点主文件
```

## 编译方法

在工作空间根目录下执行：

```bash
cd 项目根目录
colcon build --packages-select core_mini
source install/setup.bash
```

## 运行方法

### 方式1：使用launch文件
```bash
ros2 launch core_mini core_mini_launch.py
```

### 方式2：单独运行节点
```bash
# 终端1：启动MAVROS
ros2 run mavros mavros_node --ros-args -p fcu_url:=serial:///dev/ttyACM0:921600

# 终端2：启动激光雷达相关节点
# ... (需要启动相关驱动）

# 终端3：启动控制节点
ros2 run core_mini quad_mini_node
```

## 代码结构说明

### QuadcopterMini类（主节点类）

这个类整合了原来的`flight_controller`和`quadcopter`两个类的功能：

**公开成员变量：**
- `x, y, z, yaw`：飞行器当前位置和姿态

**主要方法：**
- `start_spin_thread()`：启动消息处理线程
- `prepare_for_flight()`：起飞前检查和准备
- `fly_to_target()`：定点飞行
- `fly_by_velocity()`：速度控制
- `run_mission()`：主任务循环（可自定义）

### 数据类型

**Target类（目标点）：**
```cpp
Target target(1.0, 2.0, 0.5, 0.0);  // x=1m, y=2m, z=0.5m, yaw=0rad
```

**Velocity类（速度）：**
```cpp
Velocity vel(0.2, 0.0, 0.0, 0.0);  // vx=0.2m/s, 其他为0
```

## 修改任务逻辑

要修改飞行任务，只需编辑`quadcopter_mini.cpp`中的`run_mission()`函数：

```cpp
void run_mission() {
    // 你的任务代码
    Target point1(1.0, 0.0, 0.5, 0.0);
    fly_to_target(point1);
    
    // 更多任务...
}
```

## 学习建议

1. **从simple_types.hpp开始**：理解Target和Velocity类的结构
2. **阅读构造函数**：了解节点如何初始化（订阅、发布、服务）
3. **理解飞行流程**：
   - main() → start_spin_thread() → prepare_for_flight() → run_mission()
4. **实验fly_to_target()**：修改目标点，观察飞行效果
5. **尝试速度控制**：取消注释run_mission()中的速度控制示例

## 与完整版core的区别

| 特性 | core（完整版） | core_mini（教学版） |
|------|---------------|-------------------|
| 类结构 | 分离的飞控类和实体类 | 合并到一个类 |
| PID控制 | ✅ | ❌ |
| 航点飞行 | ✅ | ❌ |
| 数据封装 | 私有变量+get/set | 公开变量 |
| 代码注释 | 较少 | 非常详细 |
| 复杂度 | 高 | 低 |

## 常见问题

**Q: 为什么要延迟启动控制节点？**  
A: 需要等待激光雷达定位系统完全就绪，否则位置数据可能不准确。

**Q: 如何调整飞行速度？**  
A: 修改`fly_to_target()`中的timeout和check_distance参数，或者直接使用`fly_by_velocity()`控制速度。

**Q: 飞行器不起飞怎么办？**  
A: 检查：1) MAVROS连接是否正常 2) 激光雷达数据是否接收 3) 解锁是否成功

## 依赖项

- ROS2 (Humble或更高版本)
- MAVROS
- ros2_tools（自定义激光雷达工具包）
- livox_ros_driver2（激光雷达驱动）
- fast_lio（激光雷达里程计）

## 作者

Hiromichi123

## 更新日志

- 2026-01-16：初始版本发布
