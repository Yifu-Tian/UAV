# Prometheus代码框架
![1223322324253432002740224](https://github.com/Travis-ovo/UAV/assets/102942951/90ed2eac-d7c5-4e0e-a198-2c4bcdb845cf)

Prometheus代码框架主要包含Experiment、Scripts、Simulator、Modules四个模块：

- Experiment

  与阿木实验室Z410无人机配套的真机代码，其余安装有Prometheus项目的无人机也可以通过该模块内容适配Prometheus项目，但可能需要修改部分参数，详情请查看Prometheus真机教程。

- Modules

  Modules作为Prometheus项目最重要的组成部分，包含各个功能子模块的全部源代码，主要包括通用子模块、控制子模块、教程子模块、目标检测子模块、规划子模块等功能源代码。

- Scripts

  Scripts下有installation和simulation两大部分，其中installation包含目标检测子模块环境安装脚本以及prometheus_mavros安装脚本，simulation包含各仿真功能启动脚本以及测试脚本。

- Simulator

  提供基于PX4-Gazebo的Prometheus仿真代码，包含无人机、传感器、二维码以及环境等相关模型，控制插件以及仿真launch文件等。

