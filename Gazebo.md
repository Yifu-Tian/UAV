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

```




对于机器人模型(urdf描述), 需要对每一个link添加<gazebo>标签, 才能让模型在Gazebo仿真环境中动起来
