# URDF
## 命令行工具
在终端中独立安装`liburdfdom-tools`, 用来检查, 梳理模型文件
```
sudo apt-get install linurdfdom-tools
```
然后使用check_urdf命令对urdf模型文件进行检查

### 问题  
可能会出现如下报错(右下为正常)
![image](https://github.com/Travis-alt/UAV/assets/102942951/f53e05fb-aac5-4cbe-8758-b436355bb0be)
### 解决  
在模型文件所在的目录下打开终端再输入命令即可解决  

还可以使用`urdf_to_graphiz`命令查看URDF模型的整体结构
```
urdf_to_graphiz mrobot_chassis.urdf
```
## 在rviz中显示模型
使用rviz将模型可视化显示出来
需要创建用于显示模型的launch文件, 如diaplay_mrobot_chassis_urdf.launch(放在功能包的launch文件夹中)
```launch
<launch>
	<arg name="model" default="$(find xacro)/xacro --inorder '$(find mrobot_description)/urdf/mrobot_with_rplidar.urdf.xacro'" />
	<arg name="gui" default="true" />

	<param name="robot_description" command="$(arg model)" />

    <!-- 设置GUI参数，显示关节控制插件 -->
	<param name="use_gui" value="$(arg gui)"/>

    <!-- 运行joint_state_publisher节点，发布机器人的关节状态  -->
	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />

	<!-- 运行robot_state_publisher节点，发布tf  -->
	<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

    <!-- 运行rviz可视化界面 -->
	<node name="rviz" pkg="rviz" type="rviz" args="-d $(find mrobot_description)/config/mrobot.rviz" required="true" />

</launch>
```
在终端中运行该launch文件
### 问题
出现RLE报错
![Screenshot from 2023-06-28 12-16-04](https://github.com/Travis-alt/UAV/assets/102942951/a4f6453a-f522-454a-8018-c2197ac0f5b4)
### 解决
问题在于, 不能简单复制别人的功能包就完事了. 
首先需要把功能包放在工作空间的src文件夹中, 然后回到工作空间根目录下编译
```
catkin_make
source devel/setup.bash
```
最后再在终端中执行之前的命令即可

## 改进URDF模型

添加物理属性和碰撞属性, 加入<inertia>和<collision>标签
### 使用xacro优化urdf
- xacro是一个精简版本的URDF文件, 在xacro文件中, 可以通过创建宏定义的方式定义常量或复用代码, 可以减少代码量, 让模型代码具有可读性
- xacro的语法支持一些可编程接口, 如常量, 变量, 数学公式等

### 使用常量定义
xacro提供了一种常量属性的定义方式:
```xacro
<xacro:property name="M_PI" value="3.14159" />
```
如果要使用该常量, 使用如下语法调用:
```xacro
<origin xyz="0 0 0" rpy="${M_PI/2} 0 0" />
```
如果需要对机器人模型进行调参,只需要修改这些参数即可
### 使用宏定义
xacro文件可以使用宏定义来声明重复使用的代码模块

## 在rviz中显示优化后的模型
1. 将xacro文件转换成URDF文件
2. 直接调用xacro文件解析器
   在launch文件中调用xacro解析器, 自动将xacro转换成URDF文件
   ```
   roslaunch mrobot_description display_mrobot.launch
   ```
![Screenshot from 2023-06-28 14-44-12](https://github.com/Travis-alt/UAV/assets/102942951/eb3bcdfb-995a-45ed-89be-8a66d1d923d2)

对于机器人模型(urdf描述), 需要对每一个link添加<gazebo>标签, 才能让模型在Gazebo仿真环境中动起来
# Gazebo
## 在Gazebo中显示模型
创建启动文件view_mrobot_gazebo.launch
在终端中编译后, 输入命令
```
roslaunch mrobot_gazebo view_mrobot_gazebo.launch
```
## 控制机器人在Gazebo中的运动
发布键盘控制节点
```
roslaunch mrobot_teleop mrobot_teleop.launch
```
### 问题
可能会遇到如下报错
![image](https://github.com/Travis-alt/UAV/assets/102942951/e7d1d7b2-8762-486c-920a-050a48f25e68)
### 解决
需要把mrobot_teleop.py改成可执行文件, 右键文件进入属性, 在权限的Execute:Allow execute file as program打勾即可
