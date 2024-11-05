#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time

# Global variable to store odometry data
current_position = None

# Callback function to update the robot's position
def odom_callback(msg):
    global current_position
    current_position = msg.pose.pose.position
    rospy.loginfo("Position - X: %f, Y: %f, Z: %f", current_position.x, current_position.y, current_position.z)

# Function to move the robot in a triangle pattern
def move_triangle():
    rospy.init_node('move_triangle', anonymous=True)
    
    # Subscriber to odometry topic
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    # Publisher to cmd_vel topic to send movement commands
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz for rate control
    move_cmd = Twist()

    # Wait for the first odom data
    while current_position is None:
        rospy.loginfo("Waiting for odometry data...")
        rospy.sleep(1)

    rospy.loginfo("Starting to move in a triangle...")

    for _ in range(3):  # Three sides of the triangle
        # Move forward
        move_cmd.linear.x = 0.5
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.loginfo("Moving forward at 0.5 m/s")
        time.sleep(2)  # Move for 2 seconds
        
        # Stop before turning
        move_cmd.linear.x = 0.0
        pub.publish(move_cmd)
        rospy.loginfo("Stopping before turn")
        time.sleep(1)

        # Rotate by 120 degrees (triangle angle)
        move_cmd.angular.z = math.radians(120)  # Convert degrees to radians
        pub.publish(move_cmd)
        rospy.loginfo("Turning by 120 degrees")
        time.sleep(1)

    # Stop after completing triangle movement
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rospy.loginfo("Movement completed. Stopping the robot.")

    # Final position info
    rospy.loginfo("Final position: X: %f, Y: %f, Z: %f", current_position.x, current_position.y, current_position.z)

if __name__ == '__main__':
    try:
        move_triangle()
    except rospy.ROSInterruptException:
        pass
