#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time

# Global variable to store the robot's position and orientation
current_position = None

# Callback function to update position and orientation from the Odometry message
def odom_callback(msg):
    global current_position
    # Store the robot's position (x, y, z) and orientation (quaternion)
    current_position = msg.pose.pose
    # Extract position and orientation
    position = current_position.position
    orientation = current_position.orientation
    # Log the current position and orientation
    rospy.loginfo("Current Position - X: %f, Y: %f, Z: %f", position.x, position.y, position.z)
    rospy.loginfo("Orientation - Quaternion: [x: %f, y: %f, z: %f, w: %f]", orientation.x, orientation.y, orientation.z, orientation.w)

# Function to move the robot in a figure-eight pattern
def move_figure_eight():
    rospy.init_node('move_figure_eight', anonymous=True)

    # Subscriber to odometry topic
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Publisher to send movement commands
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz for rate control
    move_cmd = Twist()

    # Wait for the first odometry message to be received
    while current_position is None:
        rospy.loginfo("Waiting for odometry data...")
        rospy.sleep(1)

    rospy.loginfo("Starting to move in a figure-eight pattern...")

    for _ in range(2):  # Two loops of figure-eight pattern
        # First loop (left curve)
        move_cmd.linear.x = 0.5
        move_cmd.angular.z = math.radians(45)  # Turn left gradually
        pub.publish(move_cmd)
        rospy.loginfo("Moving in left curve (linear speed: 0.5 m/s, angular speed: 45 degrees/s)")
        time.sleep(4)  # Move for 4 seconds
        
        # Second loop (right curve)
        move_cmd.angular.z = -math.radians(45)  # Turn right gradually
        pub.publish(move_cmd)
        rospy.loginfo("Moving in right curve (linear speed: 0.5 m/s, angular speed: -45 degrees/s)")
        time.sleep(4)

    # Stop the robot after completing the figure-eight
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rospy.loginfo("Figure-eight movement completed. Stopping the robot.")

    # Final position and orientation info
    rospy.loginfo("Final Position - X: %f, Y: %f, Z: %f", current_position.position.x, current_position.position.y, current_position.position.z)
    rospy.loginfo("Final Orientation - Quaternion: [x: %f, y: %f, z: %f, w: %f]", current_position.orientation.x, current_position.orientation.y, current_position.orientation.z, current_position.orientation.w)

if __name__ == '__main__':
    try:
        move_figure_eight()
    except rospy.ROSInterruptException:
        pass
