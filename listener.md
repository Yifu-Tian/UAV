# Introduction
This works as a **subscriber template** to better understand how a subscriber is created and works. Don't forget to take a look if you get into dilemma.

# Main Steps
1. Initialize ROS nodes
2. Subscribe topics
3. Wait for the topic messages, callback if received.

# Program
```C++
#include "ros/ros.h"
#include "std_msgs/String.h"

// definition of chatterCallback
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  // Handle the messages received
  // Print out the message 
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  
  ros::spin();
  
  return 0;
}
```
