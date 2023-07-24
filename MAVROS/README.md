## 什么是ROS?
ROS是一个适用于机器人的开源的元操作系统, 其主要目标是为机器人研究和开发提供代码复用的支持, 提供类似操作系统所提供的功能.  
## 什么是MAVROS?
mavros是ros的一个软件包, **用于将ROS和MAVLink协议连接起来**, 允许在运行ros的计算机, 支持MVLink的飞控板以及支持MAVLink的地面站之间通讯.  
### MAVLink
MAVLink由17个字节组成, 包括消息ID, 目标ID和数据  
![MAVROS_Message](https://github.com/TTrravis/UAV/assets/102942951/f94c4320-0abc-47ae-be2d-0ad0cfff688f)  
这使得MAVLink能够从同一个通道传输信息, 从多个无人机获取信息; 消息也可以通过无线信号进行传输  
![mavros_raw](https://github.com/TTrravis/UAV/assets/102942951/9aa52fc1-7985-4a0a-9610-dd49745bb5e2)  
mavros用于无人机通信, 可以将飞控与主控的信息进行交换.
