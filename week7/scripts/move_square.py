#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
import time

# Function to move the robot in a square
def move_square():
    rospy.init_node('move_square', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    move_cmd = Twist()

    for _ in range(4):  # Four sides of the square
        # Move forward
        move_cmd.linear.x = 0.5
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        time.sleep(2)  # Move for 2 seconds
        
        # Stop before turning
        move_cmd.linear.x = 0.0
        pub.publish(move_cmd)
        time.sleep(1)
        
        # Rotate 90 degrees
        move_cmd.angular.z = math.radians(90)
        pub.publish(move_cmd)
        time.sleep(1)  # Rotate for 1 second
        
        # Stop rotating
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        time.sleep(1)
