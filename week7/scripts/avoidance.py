#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotController:
    def __init__(self):
        # Initialize the node
        rospy.init_node('robot_controller', anonymous=True)

        # Publisher to send velocity commands to the robot
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to read LaserScan data
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # Create a Twist message for movement commands
        self.move_cmd = Twist()

        # Flag to control movement based on obstacle detection
        self.obstacle_detected = False

        # Set the rate of the loop
        self.rate = rospy.Rate(10)  # 10 Hz

    def laser_callback(self, data):
        # Filter LaserScan data to avoid invalid readings (Inf or NaN)
        valid_ranges = [r for r in data.ranges if r != float('Inf') and r != float('NaN')]

        # Define the range indices we care about (e.g., front 10 degrees)
        num_ranges = len(valid_ranges)
        front_range = valid_ranges[num_ranges//2 - 5 : num_ranges//2 + 5]  # Checking 10 readings directly ahead

        # Check if any range in front is less than 0.5 meters
        if min(front_range) < 0.5:
            self.obstacle_detected = True  # Obstacle detected
        else:
            self.obstacle_detected = False  # No obstacle

    def move_forward(self):
        # Move the robot forward unless an obstacle is detected
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                # Stop the robot if an obstacle is detected
                self.move_cmd.linear.x = 0.0  # Stop moving forward
                self.move_cmd.angular.z = 0.0  # No rotation
                rospy.loginfo("Obstacle detected in front! Stopping the robot.")
            else:
                # Move the robot forward if no obstacle is detected
                self.move_cmd.linear.x = 0.2  # Move forward at 0.2 m/s
                self.move_cmd.angular.z = 0.0  # No rotation
                rospy.loginfo("Moving forward.")

            # Publish the movement command
            self.velocity_publisher.publish(self.move_cmd)

            # Sleep to maintain loop rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Instantiate the controller object and start moving the robot
        controller = RobotController()
        controller.move_forward()
    except rospy.ROSInterruptException:
        pass

