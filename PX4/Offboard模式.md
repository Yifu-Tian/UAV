
# 什么是offboard模式
- offboard是一种无人机的飞行模式, 主要用于控制飞机运动和姿态, 目前仅支持MAVLink消息的一个小部分.
- 通过遥控器切入offboard模式, 这时无人机只听从板载计算机发来的mavlink消息, 必须遥控器切出iffboard模式, 遥控器才能换为遥控器控制无人机
- 切换至offboard模式后, 板载计算机发送指令无人机将会响应
- offboard模式需要主动连接到远程MAVLink系统

# 官方offboard例程
1. 初始化一系列需要的对象和数据
2. 给飞控发送一百个数据，用于激活（因为想转到OFFBOARD模式需要先发送一百次数据）
3. 发送模式设置请求，如果模式设置通过，则打印信息：OFFBOARD

- 外层  
<img width="197" alt="image" src="https://github.com/TTrravis/UAV/assets/102942951/813f1a7c-bee7-4d2b-b3b3-121a69dd77f2">

- 主循环  
<img width="356" alt="image" src="https://github.com/TTrravis/UAV/assets/102942951/024d34f0-aa40-4380-9575-4bc3756430ba">

# Program implemented by C++
```C++
//首先导入一系列头文件
#include <ros/ros.h>//ros库
#include <geometry_msgs/PoseStamped.h>  
//发布的位置消息体对应的头文件，该消息体的类型为geometry_msgs：：PoseStamped
//用来进行发送目标位置

#include <mavros_msgs/CommandBool.h>  
//CommandBool服务的头文件，该服务的类型为mavros_msgs：：CommandBool
//用来进行无人机解锁

#include <mavros_msgs/SetMode.h>     
//SetMode服务的头文件，该服务的类型为mavros_msgs：：SetMode
//用来设置无人机的飞行模式，切换offboard

#include <mavros_msgs/State.h>  
//订阅的消息体的头文件，该消息体的类型为mavros_msgs：：State
//查看无人机的状态
 
//建立一个订阅消息体类型的变量，用于存储订阅的信息
mavros_msgs::State current_state;
 
//订阅时的回调函数，接受到该消息体的内容时执行里面的内容，这里面的内容就是赋值
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
 
 
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node"); //ros系统的初始化，最后一个参数为节点名称
    ros::NodeHandle nh;
 
    //订阅。<>里面为模板参数，传入的是订阅的消息体类型，（）里面传入三个参数，分别是该消息体的位置、缓存大小（通常为1000）、回调函数
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
 
    //发布之前需要公告，并获取句柄，发布的消息体的类型为：geometry_msgs::PoseStamped
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
 
    //启动服务1，设置客户端（Client）名称为arming_client，客户端的类型为ros::ServiceClient，
    //启动服务用的函数为nh下的serviceClient<>()函数，<>里面是该服务的类型，（）里面是该服务的路径
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
 
    //启动服务2，设置客户端（Client）名称为set_mode_client，客户端的类型为ros::ServiceClient，
    //启动服务用的函数为nh下的serviceClient<>()函数，<>里面是该服务的类型，（）里面是该服务的路径
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
 
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
 
    // 等待飞控连接mavros，current_state是我们订阅的mavros的状态，连接成功在跳出循环
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
 
 
    //先实例化一个geometry_msgs::PoseStamped类型的对象，并对其赋值，最后将其发布出去
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
 
    //建立一个类型为SetMode的服务端offb_set_mode，并将其中的模式mode设为"OFFBOARD"，作用便是用于后面的
    //客户端与服务端之间的通信（服务）
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    //建立一个类型为CommandBool的服务端arm_cmd，并将其中的是否解锁设为"true"，作用便是用于后面的
    //客户端与服务端之间的通信（服务）
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
 
    //更新时间
    ros::Time last_request = ros::Time::now();
 
    while(ros::ok())//进入大循环
    {
        //首先判断当前模式是否为offboard模式，如果不是，则客户端set_mode_client向服务端offb_set_mode发起请求call，
        //然后服务端回应response将模式返回，这就打开了offboard模式
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");//打开模式后打印信息
            }
            last_request = ros::Time::now();
        }
        else //else指已经为offboard模式，然后进去判断是否解锁，如果没有解锁，则客户端arming_client向服务端arm_cmd发起请求call
            //然后服务端回应response成功解锁，这就解锁了
        {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");//解锁后打印信息
                }
                last_request = ros::Time::now();
            }
        }
 
        local_pos_pub.publish(pose); 
        //发布位置信息，所以综上飞机只有先打开offboard模式然后解锁才能飞起来
 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}
```
# Analysis
## 头文件分析
```C++
#include <ros/ros.h>//ros库
#include <geometry_msgs/PoseStamped.h>  
//发布的位置消息体对应的头文件，该消息体的类型为geometry_msgs：：PoseStamped
//用来进行发送目标位置
/*
ros官网上这样定义
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose
实际上就是一个带有头消息和位姿的消息
*/

#include <mavros_msgs/CommandBool.h>  
/*
CommandBool服务的头文件，该服务的类型为mavros_msgs：：CommandBool
其结构如下（来源于ros wiki）
# Common type for switch commands

bool value
---
bool success
uint8 result

可以看到，发送的请求是一个bool类型的数据，为True则解锁，为False则上锁
返回的响应中
success是一个bool类型的参数，表示上电/断电操作是否成功执行。
如果操作成功执行，success值为True，否则为False。
result是一个int32类型的参数，表示执行上电/断电操作的结果。
如果解锁/上锁操作成功执行，result值为0，
否则为其他值，表示执行解锁/上锁操作时发生了某种错误或异常。可以根据这个数值查看是哪种问题导致
*/
//用来进行无人机解锁

#include <mavros_msgs/SetMode.h>     
//SetMode服务的头文件，该服务的类型为mavros_msgs：：SetMode
//用来设置无人机的飞行模式，切换offboard
/*
wiki上的消息定义如下
# set FCU mode
#
# Known custom modes listed here:
# http://wiki.ros.org/mavros/CustomModes

# basic modes from MAV_MODE
uint8 MAV_MODE_PREFLIGHT = 0
uint8 MAV_MODE_STABILIZE_DISARMED = 80
uint8 MAV_MODE_STABILIZE_ARMED = 208
uint8 MAV_MODE_MANUAL_DISARMED = 64
uint8 MAV_MODE_MANUAL_ARMED = 192
uint8 MAV_MODE_GUIDED_DISARMED = 88
uint8 MAV_MODE_GUIDED_ARMED = 216
uint8 MAV_MODE_AUTO_DISARMED = 92
uint8 MAV_MODE_AUTO_ARMED = 220
uint8 MAV_MODE_TEST_DISARMED = 66
uint8 MAV_MODE_TEST_ARMED = 194

uint8 base_mode # filled by MAV_MODE enum value or 0 if custom_mode != ''
string custom_mode # string mode representation or integer
---
bool success

实际上String类型的变量custom_mode就是我们想切换的模式，有如下选择
MANUAL，ACRO，ALTCTL，POSCTL，OFFBOARD，STABILIZED，RATTITUDE，AUTO.MISSION
AUTO.LOITER，AUTO.RTL，AUTO.LAND，AUTO.RTGS，AUTO.READY，AUTO.TAKEOFF
*/
#include <mavros_msgs/State.h>  
//订阅的消息体的头文件，该消息体的类型为mavros_msgs：：State
//用于描述无人机当前状态的各种参数
/*
wiki上是这样的
std_msgs/Header header
bool connected
bool armed
bool guided
bool manual_input
string mode
uint8 system_status
PS：后面还有一堆描述无人机状态的，我这里并没有写，因为一般用不到，感兴趣的可以去wiki上看
http://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/State.html
解析如下：
header：消息头，包含时间戳和框架信息；
connected：表示是否连接到了 mavros 节点；
armed：表示无人机当前是否上锁；
guided：表示无人机当前是否处于 GUIDED 模式；
mode：表示当前无人机所处的模式，包括以下几种：
*/
```
## 流程分析
```C++
    //这是一个订阅者对象，可以订阅无人机的状态信息（状态信息来源为mavros发布），用来判断无人机的状态
    //程序在最开始的时候声明了一个全局变量，用来存储无人机状态，在回调函数里面会不断更新这个状态变量
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
 
    //用来在本地坐标系下发布目标点
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
 
    //一个客户端，用来解锁无人机，这是因为无人机如果降落后一段时间没有收到信号输入，会自动上锁来保障安全
    //所以如果想让无人机飞行，必须使用这个实现解锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
 
    //因为无人机有多种飞行模式，所以需要程序运行时进行切换
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Rate rate(20.0);//因为无人机在空中飞行，更难以控制，所以要求信号的频率较高
    // 等待飞控连接mavros，current_state是我们订阅的mavros的状态，连接成功在跳出循环
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    //大家还记得头文件里面mavros_msgs/State.h吗？这个消息格式里面有很多属性可以说明无人机的状态
    //加上我们创建一个全局变量来不断监视无人机状态，在这里我们就可以查看无人机的连接状态

```
## 人话
该官方例程的主要功能是: 使无人机缓慢飞到2米的高度

# 实操
