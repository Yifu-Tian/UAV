# URDF
在终端中独立安装`liburdfdom-tools`, 用来检查, 梳理模型文件
```
sudo apt-get install linurdfdom-tools
```
然后使用check_urdf命令对urdf模型文件进行检查
## 问题
可能会出现如下报错







对于机器人模型(urdf描述), 需要对每一个link添加<gazebo>标签, 才能让模型在Gazebo仿真环境中动起来
