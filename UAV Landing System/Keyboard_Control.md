# Introduction
&emsp;As you see in the title, this file mainly introduce a python script to simplify the commands entered in the Linux terminal.  
&emsp;In the code, "K_UP" implies "rise", "K_DOWN" implies "drop", "K_RIGHT" implies "open", "K_LEFT" implies "align", "K_q" implies quit.
# Program
```Python
#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
import sys
import pygame
from pygame.locals import *

from pygame.locals import *
from sys import exit



class Keyboard():
    def __init__(self):
        rospy.init_node("Key_control_node", anonymous=True)
        self.pub = rospy.Publisher("message",String, queue_size=10)
        self.rate = rospy.Rate(1)

    def drop(self):
        if not rospy.is_shutdown():
            msg = "down"
            self.pub.publish(msg)    

    def rise(self):
        if not rospy.is_shutdown():
            msg = "up"
            self.pub.publish(msg)

    def align(self):
        if not rospy.is_shutdown():
            msg = "align"
            self.pub.publish(msg)    

    def open(self):
        if not rospy.is_shutdown():
            msg = "open"
            self.pub.publish(msg)   
    

if __name__ == "__main__":

    myKeyboard = Keyboard()
    
    pygame.init()
    display = pygame.display.set_mode((300,300))

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:                # if the event is about quitting the system
                pygame.quit()
                sys.exit()
            
            if event.type == pygame.KEYDOWN:       # if the evnet is about pressing keys
                if event.key == pygame.K_UP:
                    myKeyboard.rise()
                if event.key == pygame.K_DOWN:
                    myKeyboard.drop()
                if event.key == pygame.K_LEFT:
                    myKeyboard.open()
                if event.key == pygame.K_RIGHT:
                    myKeyboard.align()
                if event.key == pygame.K_q:
                    pygame.quit()
                    sys.exit()
```
