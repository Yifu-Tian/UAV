# 升级cmake版本
[备忘录 linux升级cmake(3.13.2) - 高隽睿志的文章 - 知乎](https://zhuanlan.zhihu.com/p/348384801)

# Prometheus文件夹下运行全部编译脚本报错
![d9b5030527d4f16365be2011e529d5b](https://github.com/Travis-ovo/UAV/assets/102942951/d253bb1b-30e1-4049-88df-6a9ed4d8d24e)
- 解决
  安装multi_map_server的ROS插件 `sudo apt-get install ros-melodic-multi-map-server`
- 另外
  Prometheus包放在home下

# Prometheus代码框架
![Screenshot from 2023-07-07 16-26-22](https://github.com/Travis-ovo/UAV/assets/102942951/558291a7-2e5a-47ef-852a-0e8c19784c4f)

- Modules: 各模块功能的源代码
- Simulator: 仿真飞行相关代码
- Experiment: 真实飞行相关代码

# PX4编译环境配置及固件代码安装
Prometheus项目中的Gazebo仿真模块依赖PX4固件及sitl_gazebo包，因此需先配置PX4编译环境。
![Screenshot from 2023-07-07 18-31-09](https://github.com/Travis-ovo/UAV/assets/102942951/1fcec2f2-1422-4788-b914-a42ca55e7969)
