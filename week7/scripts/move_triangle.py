#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
import time

# Function to move the robot in a triangle pattern
def move_triangle():
    rospy.init_node('move_triangle', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1) 
    move_cmd = Twist()

    for _ in range(3):  # Three sides of the triangle
        # Move forward
        move_cmd.linear.x = 0.5
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        time.sleep(2)  # Move for 2 seconds
        
        # Stop before turning
        move_cmd.linear.x = 0.0
        pub.publish(move_cmd)
        time.sleep(1)

        # Rotate by 120 degrees (triangle angle)
        move_cmd.angular.z = math.radians(120)
        pub.publish(move_cmd)
        time.sleep(1)

    # Stop after completing triangle movement
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)


