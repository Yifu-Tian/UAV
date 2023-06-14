# Introduction
Due to the need of research topic, I get down to the ROS and ESP32 these days. Now I have just finished some basic operations based on ROS and multiple ESP32. 
This passage tells the steps and some problems I met during the process.
# Connection of ROS with Single ESP32

## Preparation
1. The computer operating system I used is Ubuntu 18.04.
2. Environment set up is not the keystone in this passage. rosserial_arduino library is necessary and important. So make sure you have downloaded the package. 

3. ESP32 *2

4. The ssid and password of WiFi(the host WiFi). Please note that ESP32 must be on the same network segment as the host.

5. The IP address of host. In Linux, you can enter `ifconfig` command in the terminal to check the IP address. 
## Program
We can use the example program in rosserial_arduino library.
```C
#include <WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>


const char* ssid     = "LAB202";
const char* password = "114514";

// Set the rosserial socket server IP address
IPAddress server(11,4,5,14);

// Set the rosserial socket server port
const uint16_t serverPort = 11411;


ros::NodeHandle nh;
// Make a chatter publisher
std_msgs::String str_msg;
ros::Publisher chatter("esp32dev", &str_msg); // esp32dev play the role of topic name. You can rename it whatever

// say hello
char hello[13] = "hello world!";

void setup()
{
  // Use ESP8266 serial to monitor the process
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
 
  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
	// Well this may lead to endless dot if connection doesn't work. 
	// It would be better to count the number of dot and pause the while loop when reached the specific number.
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
 
  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
 
  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());
 
  // Start to be polite
  nh.advertise(chatter);
}
 
void loop()
{
 
  if (nh.connected()) {
    Serial.println("Connected");
    // Say hello
    str_msg.data = hello;
    chatter.publish( &str_msg );
  } else {
    Serial.println("Not Connected");
  }
  nh.spinOnce();
  // Loop exproximativly at 1Hz
  delay(1000);
}
```
## Launch on the Host Computer
Open terminal and enter
`roscore`
Then enter
`rosrun rosserial_python serial_node.py tcp`
Enter `rostopic list` to check the topic esp32 have published. 
Also, you can enter `rostopic echo` to check the content of topic if your program include this.

# Connection of ROS with Multiple ESP32

## ESP32_1 & ESP32_2
We keep ESP32_1 the same as before. In ESP32_2, we modify some places:
1. Change the Port number to another. It should differentiate with ESP32_1. The default port number is 11411, so in ESP32_2 we change it into 11412.
2. Change the name of topic, since ROS doesn't allow the existence of two same topic name.

Here I post the program of ESP32_2
```C
#include <WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
 
const char* ssid     = "LAB202";
const char* password = "114514";
// Set the rosserial socket server IP address
IPAddress server(11,4,5,14);
// Set the rosserial socket server port
const uint16_t serverPort = 11412;
 
ros::NodeHandle nh;
// Make a chatter publisher
std_msgs::String str_msg;
ros::Publisher chatter("esp32pico", &str_msg);
 
// Be polite and say hello
char hello[13] = "hello world!";
 
void setup()
{
  // Use ESP8266 serial to monitor the process
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
 
  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
 
  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
 
  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());
 
  // Start to be polite
  nh.advertise(chatter);
}
 
void loop()
{
 
  if (nh.connected()) {
    Serial.println("Connected");
    // Say hello
    str_msg.data = hello;
    chatter.publish( &str_msg );
  } else {
    Serial.println("Not Connected");
  }
  nh.spinOnce();
  // Loop exproximativly at 1Hz
  delay(1000);
}
```
## Launch on the Host Computer
Write a launch file to run multiple ros nodes. Please make sure the launch file should be in the package folder.
```launch
<launch>
<node pkg="rosserial_python" type="serial_node.py" name="ESP32DEV" args="tcp 11411"/>
<node okg="rosserial_python" type="serial_node.py" name="ESP32PCIO" args="tcp 11412"/>
</launch>
```
**Do not forget catkin_make in the catkin_ws root directory**

Now open the terminal and enter `roslaunch [package name] [launchfile.launch]`  
Enter `rostopic list` to check the published topic  
Enter `rostopic ehco /esp32dev` to check the topic content of esp32dev  
Enter `rostopic echo /esp32pico` to check the topic content of esp32pico
## Problem
- Error when roslaunch in the terminal
	Solution: First make sure you are in the catkin_ws root directory. Next make sure you have compiled all the workspace. Any new added files should be compiled, too.
	```
	$ cd ~/catkin_ws/
	$ catkin_make
	$ source devel/setup.bash
	```
# Reference
- [1][【esp32&ROS】ROS与多个ESP32的无线连接](https://blog.csdn.net/weixin_43326110/article/details/127695803)
