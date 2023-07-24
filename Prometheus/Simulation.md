# 参考
[Prometheus使用手册](https://wiki.amovlab.com/public/prometheus-wiki/)

# 升级cmake版本
[备忘录 linux升级cmake(3.13.2) - 高隽睿志的文章 - 知乎](https://zhuanlan.zhihu.com/p/348384801)

# Prometheus文件夹下运行全部编译脚本报错
![d9b5030527d4f16365be2011e529d5b](https://github.com/Travis-ovo/UAV/assets/102942951/d253bb1b-30e1-4049-88df-6a9ed4d8d24e)
- 解决
  安装multi_map_server的ROS插件 `sudo apt-get install ros-melodic-multi-map-server`
- 另外Prometheus包放在home下

# Prometheus代码框架
![Screenshot from 2023-07-07 16-26-22](https://github.com/Travis-ovo/UAV/assets/102942951/558291a7-2e5a-47ef-852a-0e8c19784c4f)

- Modules: 各模块功能的源代码
- Simulator: 仿真飞行相关代码
- Experiment: 真实飞行相关代码

# PX4编译环境配置及固件代码安装
Prometheus项目中的Gazebo仿真模块依赖PX4固件及sitl_gazebo包，因此需先配置PX4编译环境。
![Screenshot from 2023-07-07 18-31-09](https://github.com/Travis-ovo/UAV/assets/102942951/1fcec2f2-1422-4788-b914-a42ca55e7969)

## git tag报错
怀疑可能是clone github导致标签不匹配, 更换为clone gitee到本地, 再重复编译流程即可

## 编译通过了, 但model迟迟无法加载
- 问题就是网络问题, 需要把~/prometheus_px4/Tools/models中的所有文件复制到~/.gazebo/models文件夹中
- 注意.gazebo文件夹是隐藏文件夹, 按ctrl+h即可
- .gazebo中如果没有models文件夹就创建一个, 然后复制进去即可

# 编译成功效果图
久违的成功, 途中遇到大大小小问题还是很折磨人的, 做到这里先给自己点个赞ovo!  
(注: 无人机太黑了可能看不到)  

![Screenshot from 2023-07-18 15-28-51](https://github.com/Travis-ovo/UAV/assets/102942951/ca7dcf4b-813d-4e90-83cb-1aba83279f65)

# Prometheus_mavros安装问题
## ./install_prometheus_mavros.sh命令报错
![Screenshot from 2023-07-19 10-36-32](https://github.com/Travis-ovo/UAV/assets/102942951/21338911-ab1d-42ff-8114-8059f0412a06)  
如上图, 报错信息为: Error: the following packages/stacks could not have their rosdep keys resolved to system dependencyies  
实际上就是缺少依赖包, 在终端中输入
```
sudo apt-get install ros-[ROS版本]-缺少的依赖包
```
比如说我这里显然缺少cmake_modules和control_toolbox, 那么在终端依次输入
```
sudo apt-get install ros-melodic-cmake-modules
sudo apt-get install ros-melodic-control-toolbox
```
注意: "_"换成"-"
# 依赖项安装
## 遥控器仿真驱动安装
## Gazebo模型库下载
之前无法加载模型就是以为.gazebo文件夹中没有models文件夹, 按照官方教程创建models文件夹并添加模型文件即可
## nlink_parser安装
nlink_parser为UWB驱动功能包
## vrpn_client_ros安装
vrpn_client_ros功能包为动捕定位系统应用所依赖的ROS功能包  

以上功能包已全部安装

# 编译Prometheus
输入以下命令
```
cd Prometheus 

# 第一次使用时需要给编译脚本文件添加可执行权限
chmod +x compile_*

# 编译控制功能模块
./compile_control.sh
```

如图  
![Screenshot from 2023-07-19 11-04-05](https://github.com/Travis-ovo/UAV/assets/102942951/7f33f289-5a1a-47cd-b951-3abca305a2c7)  
目前提供四个编译脚本:  
- compile_all.sh: 编译全部功能模块
- compile_control.sh: 编译控制功能相关模块
- compile_planning.sh: 编译规划功能相关模块
- compile_detection.sh: 编译视觉功能相关模块

# 环境变量配置
打开终端输入以下命令打开.bashrc文件。
```
sudo gedit ~/.bashrc
```
将以下内容复制到.bashrc文件中后保存退出，如果.bashrc已有相关内容，则无需重复添加。
```
source {your prometheus path}/Prometheus/devel/setup.bash

export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:{your prometheus path}/Prometheus/devel/lib
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{your prometheus path}/Prometheus/Simulator/gazebo_simulator/gazebo_models/uav_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{your prometheus path}/Prometheus/Simulator/gazebo_simulator/gazebo_models/ugv_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{your prometheus path}/Prometheus/Simulator/gazebo_simulator/gazebo_models/sensor_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{your prometheus path}/Prometheus/Simulator/gazebo_simulator/gazebo_models/scene_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{your prometheus path}/Prometheus/Simulator/gazebo_simulator/gazebo_models/texture

source {your px4 path}/prometheus_px4/Tools/setup_gazebo.bash {your px4 path}/prometheus_px4 {your px4 path}/prometheus_px4/build/amovlab_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:{your px4 path}/prometheus_px4
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:{your px4 path}/prometheus_px4/Tools/sitl_gazebo
```

顺便找到了学校的位置^_^  
![Screenshot from 2023-07-19 11-33-12](https://github.com/Travis-ovo/UAV/assets/102942951/cf2f98c1-66f3-4dfe-aa21-9c3072f3928e)

# 基于PX4的旋翼无人机系统简单介绍
![image](https://github.com/Travis-ovo/UAV/assets/102942951/aff40b4d-97c7-4a5a-abbb-195f6c760354)
![1652374810652053942665216](https://github.com/Travis-ovo/UAV/assets/102942951/b3e1bbdf-bd9d-47c2-9bcf-f6bc82cac199)
Gazebo为我们提供了一个物理环境的模拟系统，例如无人机以及飞行环境的可视化、物理属性、碰撞属性等。  
基于PX4-Gazebo的仿真系统能够完全实现无人机飞行控制的全部功能，除了性能，其余方面与真机表现并无差异.可以简单理解为Gazebo提供无人机硬件仿真，而PX4提供飞控仿真，当然无人机硬件仿真也是由PX4项目组完成，Gazebo仅仅是提供一个物理仿真平台。

# 无人机控制模块&官方例程运行
- uav_control包含两个功能模块uav_controller和uav_estimator  
![1508392337346122827071488](https://github.com/Travis-ovo/UAV/assets/102942951/816fc7af-2947-4c3a-8fbc-f5435b080a79)  

- 无人机控制模块例程的运行见另一篇文章

## 终端控制台异常
根据控制台提示输入1切换为键盘控制后, 输入1切换为arm状态后输入4无法起飞, 但终端显示"Switch to Takeoff Mode"
![Screenshot from 2023-07-24 11-41-52](https://github.com/Travis-ovo/UAV/assets/102942951/2c2b7b36-d80d-4a99-a48c-a0c41a3b8239)
