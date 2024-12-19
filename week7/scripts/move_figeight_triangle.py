#!/usr/bin/env
import rospy
from geometry_msgs.msg import Twist
import math

def move_square():
    rospy.init_node('move_square', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 1 Hz

    move_cmd = Twist()
    linear_speed = 1.5  # m/s
    angular_speed = math.pi / 2  # rad/s (90 degrees per second)
    distance = 6  # meters
    move_time = distance / linear_speed  # time to move one side

    for _ in range(4):  # Four sides of the square
        # Move forward
        move_cmd.linear.x = linear_speed
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.sleep(move_time)

        # Stop
        move_cmd.linear.x = 0.0
        pub.publish(move_cmd)
        rospy.sleep(1)

        # Rotate 90 degrees
        move_cmd.angular.z = angular_speed
        pub.publish(move_cmd)
        rospy.sleep(1)

        # Stop
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.sleep(1)

    # Stop the robot
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)

def move_figure_eight():
    rospy.init_node('move_figure_eight', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    move_cmd = Twist()

    for _ in range(2):  # Two loops
        # Left curve
        move_cmd.linear.x = 0.5
        move_cmd.angular.z = math.radians(45)  # Adjust for a smoother curve
        pub.publish(move_cmd)
        rospy.sleep(4)  # Duration for one curve

        # Right curve
        move_cmd.angular.z = -math.radians(45)
        pub.publish(move_cmd)
        rospy.sleep(4)

    # Stop the robot
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)

def move_triangle():
    rospy.init_node('move_triangle', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    move_cmd = Twist()
    linear_speed = 1.5  # m/s
    angular_speed = math.pi  # rad/s
    side_length = 2  # meters
    move_time = side_length / linear_speed

    for _ in range(3):  # Three sides of the triangle
        # Move forward
        move_cmd.linear.x = linear_speed
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.sleep(move_time)

        # Stop
        move_cmd.linear.x = 0.0
        pub.publish(move_cmd)
        rospy.sleep(1)

        # Rotate 120 degrees (2π/3 radians)
        move_cmd.angular.z = angular_speed
        turn_time = (2 * math.pi / 3) / angular_speed
        pub.publish(move_cmd)
        rospy.sleep(turn_time)

        # Stop
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.sleep(1)

    # Stop the robot
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)

if __name__ == "__main__":
    try:
        move_square()
        move_figure_eight()
        move_triangle()
    except rospy.ROSInterruptException:
        pass
