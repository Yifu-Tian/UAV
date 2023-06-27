# Introduction
[ROS-based UAV Control Using Hand Gesture Recognition](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7978402)  

They design a simple but effective algorithm to distinguish different operators based on position information and color information.  
Mainly consists three components, **hand gesture recognition**, **multi-operator recognition**, **UAV control**

# Notes
- The basic states of UAV can be divided into nine types, moving forward, moving backward, moving leftward, moving rightward, ascent, descent, rotating clockwise, rotating anticlockwise and hovering.
- For UAV Control, they use ROS to send the recognition result to UAV and transfer it to control command.
- They create two nodes, namely hand gesture recognition part and UAV control part.
