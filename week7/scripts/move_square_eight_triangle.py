#!/usr/bin/env python

import time
import tf2_ros
import rospy
from geometry_msgs.msg import Twist
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Function to move the robot in the shape of a square
def move_square():
    rospy.init_node('move_square', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz loop rate
    move_cmd = Twist()

    # Define speed parameters
    linear_speed = 1.5  # meters per second (forward speed)
    angular_speed = math.pi / 2  # 90 degrees per second (angular speed)

    # Distance to move for one side of the square
    distance = 6  # meters (side length of the square)

    # Time to complete one side of the square (based on linear speed)
    move_time = distance / linear_speed  # in seconds

    for _ in range(4):  # Four sides of the square
        # Move forward for the side of the square
        move_cmd.linear.x = linear_speed
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.sleep(move_time)  # Move forward for the calculated duration

        # Stop before turning
        move_cmd.linear.x = 0.0
        pub.publish(move_cmd)
        rospy.sleep(1)  # Pause for 1 second before turning

        # Rotate 90 degrees
        move_cmd.angular.z = angular_speed
        pub.publish(move_cmd)
        rospy.sleep(1)  # 90 degrees turn takes 1 second with angular speed = math.pi/2

        # Stop rotating
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.sleep(1)  # Pause before starting the next side

        rate.sleep()  # Maintain loop rate of 1 Hz

    # Stop the robot after completing the square
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)



# Function to move the robot in a figure-eight pattern
def move_figure_eight():
    rospy.init_node('move_figure_eight', anonymous=True)

    # Publisher to send messages to the /cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Rate (in Hz) of publishing the Twist messages
    rate = rospy.Rate(1)

    # Create a Twist message object
    move_cmd = Twist()

    # Loop to repeat the figure-eight motion twice
    for _ in range(2):
        # First loop (left curve)
        move_cmd.linear.x = 0.5
        move_cmd.angular.z = math.radians(45)  # Turn left gradually
        pub.publish(move_cmd)
        time.sleep(4)  # Move for 4 seconds

        # Second loop (right curve)
        move_cmd.angular.z = -math.radians(45)  # Turn right gradually
        pub.publish(move_cmd)
        time.sleep(4)  # Move for 4 seconds

    # Stop the robot after completing the figure-eight
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)

# Function to move the robot in a triangle pattern
def move_triangle():
    rospy.init_node('move_triangle', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz loop rate
    move_cmd = Twist()

    # Define speed parameters
    linear_speed = 1.5  # meters per second (forward speed)
    angular_speed = math.pi  # radians per second (angular speed for 180 degrees per second)

    # Distance to move for one side of the triangle (2 seconds at linear_speed)
    side_length = 2  # meters (side of the triangle)

    # Time to complete one side of the triangle (based on linear speed)
    move_time = side_length / linear_speed  # time = distance / speed (in seconds)

    for _ in range(3):  # Three sides of the triangle
        # Move forward for the side of the triangle
        move_cmd.linear.x = linear_speed
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.sleep(move_time)  # Sleep for the duration to cover the side length

        # Stop before turning
        move_cmd.linear.x = 0.0
        pub.publish(move_cmd)
        rospy.sleep(1)  # Stop for 1 second

        # Rotate 120 degrees (angular speed = math.pi radians/s, 120 degrees = 2pi/3 radians)
        move_cmd.angular.z = angular_speed  # Set angular velocity for 120-degree turn
        pub.publish(move_cmd)
        # Time to complete the 120-degree turn (2π/3 radians)
        turn_time = (2 * math.pi / 3) / angular_speed  # Time = angle / angular speed
        rospy.sleep(turn_time)  # Sleep for the duration of the turn

        # Stop rotating
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.sleep(1)  # Pause for a moment before starting the next side

    # Stop after completing the triangle movement
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)


if __name__ == '__main__':
    try:
        choice = input("Enter 'square', 'figure-eight', or 'triangle': ").strip().lower()
        
        if choice == 'square':
            move_square()
        
        elif choice == 'figure-eight':
            move_figure_eight()
        
        elif choice == 'triangle':
            move_triangle()
        
        else:
            print("Invalid choice!")
            
    except rospy.ROSInterruptException:
        pass
