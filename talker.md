# Introcution
This works as a **publisher template** to better understand how a publisher is created and works.
Don't forget to take a look if you get into dilemma.

# Main Steps
1. Initialize the ROS nodes
2. Register node information to ROS Master, including topic name and messsage type in topic
3. Publish message in a given frequency

# Program
```C++
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker"); // Take care that "talker" is the node name we initialized
  
  ros::NodeHandle n;
  
  ros::Publisher chatter_pub = n.advertise<std_msgd::String>("chatter", 1000); 
  // Create a publisher "chatter_pub", and tell publisher that the node "talker" will publish String type message on the topic "chatter"
  
  ros::Rate loop_rate(10);
  
  int count = 10;
  while(ros:ok())
  {
    // jump out the loop if error occurs
    
    // intialize the message
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello" << count;
    msg.data = ss.str();
    
    // publish the message
    chatter_pub.publish(msg);
    
    ros::spinOnce();
    
    loop_rate_sleep();
    ++count;
  }
  return 0;
}
```
