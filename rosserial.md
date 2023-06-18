# Abstract
rosserial is a protocol, which is used for communication between non-ros devices and ros devices.  
It enables data interaction in non-ros environment through serial port or network.  
rosserial has **client** and **server**. The client runs in non-ros environment, and connects with rosserial server.  
The server is in ros environment, and client can publish or subscribe topics through server node.

# ROS Server
The rosserial server is a node that runs in the ROS device and serves as a serial protocol and a connection to the ROS network. Rosserial Server is available in both C++ and Python.  
## rosserial_python
serial_node.py is in the rosserial_python package. It communicates with rosserial devices through serial.
It is very common when we launch the nodes.
```
rosrun rosserial_python serial_node.py tcp
```
> TCP, transmission control protocol, 传输控制协议

## rosserial_server
rosserial_server is implemented by C++. It provides serial_node and socket_node, which is used for serial port connection and network socket connection respectively.  
```
roslaunch rosserial_server serial.launch port:=/dev/ttyUSB0
```
```
roslaunch rosserial_server socket.launch
```
# rosserial_arduino
rosserial provides a ROS communication protocol, making Arduino a ROS node.  
## Create a Publisher in rosserial_arduino
```C
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data=hello;
  chatter.publish(&std_msg);
  nh.spinOnce();
  delay(1000);
}
```
## Create a Subscriber in rosserial_arduino
```C
#include <ros.h>
#include <std_msgs/Empty.h>

void messageCb(const std_msgs::Empty &toggle_msg)
{
  digitalWrite(13, HIGH-digitalRead(13));
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscriber(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
```
