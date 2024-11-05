#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time

# Global variable to store the robot's position and orientation
current_position = None

# Callback function to update position from the Odometry message
def odom_callback(msg):
    global current_position
    # Store the robot's position and orientation (x, y, z)
    current_position = msg.pose.pose.position
    # Print the current position (x, y, z)
    rospy.loginfo("Current Position - X: %f, Y: %f, Z: %f", current_position.x, current_position.y, current_position.z)

# Function to move the robot in a square pattern
def move_square():
    rospy.init_node('move_square', anonymous=True)
    
    # Subscriber to odometry topic
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    # Publisher to send movement commands to the robot
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz for rate control
    move_cmd = Twist()

    # Wait for the first odometry message to be received
    while current_position is None:
        rospy.loginfo("Waiting for odometry data...")
        rospy.sleep(1)

    rospy.loginfo("Starting to move in a square...")

    for _ in range(4):  # Four sides of the square
        # Move forward for 2 seconds
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

        # Rotate 90 degrees (convert degrees to radians)
        move_cmd.angular.z = math.radians(90)
        pub.publish(move_cmd)
        rospy.loginfo("Turning 90 degrees")
        time.sleep(1)  # Rotate for 1 second

        # Stop rotating
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.loginfo("Stopped turning")
        time.sleep(1)

    # Stop after completing the square movement
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rospy.loginfo("Movement completed. Stopping the robot.")

    # Final position info
    rospy.loginfo("Final Position - X: %f, Y: %f, Z: %f", current_position.x, current_position.y, current_position.z)

if __name__ == '__main__':
    try:
        move_square()
    except rospy.ROSInterruptException:
        pass
