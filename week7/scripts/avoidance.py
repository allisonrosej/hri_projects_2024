#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotController:
    def __init__(self):
        # Initialize the node
        rospy.init_node('robot_controller', anonymous=True)

        # Publisher to send velocity commands
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to read LaserScan data
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # Create a Twist message for movement
        self.move_cmd = Twist()

        # Flag to control movement
        self.obstacle_detected = False

        # Set the rate of the loop
        self.rate = rospy.Rate(10)  # 10 Hz

    def laser_callback(self, data):
        # Check if there's an obstacle within 0.5 meters
        if min(data.ranges) < 0.5:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def move_forward(self):
        # Move the robot forward unless an obstacle is detected
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                self.move_cmd.linear.x = 0.0  # Stop moving
                self.move_cmd.angular.z = 0.0  # No rotation
            else:
                self.move_cmd.linear.x = 0.2  # Move forward
                self.move_cmd.angular.z = 0.0  # No rotation

            # Publish the movement command
            self.velocity_publisher.publish(self.move_cmd)

            # Sleep to maintain loop rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.move_forward()
    except rospy.ROSInterruptException:
        pass
