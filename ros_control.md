# 前言
ros-control 就是ROS为开发者提供的机器人控制中间件
# ros_control架构
- Controller Manager 控制器管理器: 管理不同控制器的通用接口
- Controller 控制器: 读取硬件资源接口中的状态, 发布控制指令. 不直接接触硬件, 从硬件抽象层请求资源
- RobotHW 机器人硬件抽象: 直接和硬件资源交互, 通过write和read 方法完成硬件操作. 管理硬件资源, 处理硬件冲突
- Hardware Resource 硬件资源:提供硬件资源的接口
# 控制器controllers
控制器可以完成每个joint的控制, 读取硬件资源接口中的状态, 再发布控制命令, 并且提供PID控制器
ros_control功能包提供的控制器种类如下
![image](https://github.com/Travis-ovo/UAV/assets/102942951/8bb002d2-3518-48fe-8667-7bb051754705)
# 硬件接口Hardware Interfaces
硬件资源接口是Controller与RobotHW交互的接口
# 控制器管理器Controller Manager
用于管理多个控制器, 实现控制器的加载, 运行, 停止等操作  
控制器管理器的输入就是ROS上层应用功能包的输出

# 创建控制器
