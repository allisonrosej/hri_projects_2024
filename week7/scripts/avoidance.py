#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('robot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.move_cmd = Twist()
        self.obstacle_detected = False

        self.rate = rospy.Rate(10) 

    def laser_callback(self, data):
        valid_ranges = [r for r in data.ranges if r != float('Inf') and r != float('NaN')]
        
        if valid_ranges:
            mid = len(valid_ranges) // 2
            front_range = valid_ranges[mid - 5 : mid + 5]
            if front_range and min(front_range) < 0.5:
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False
        else:
            self.obstacle_detected = False

    def move_forward(self):
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                # Stop the robot
                self.move_cmd.linear.x = 0.0
                self.move_cmd.angular.z = 0.0
                rospy.loginfo("Obstacle detected! Stopping the robot.")
            else:
                # Move forward
                self.move_cmd.linear.x = 0.2
                self.move_cmd.angular.z = 0.0
                rospy.loginfo("Moving forward.")
            self.velocity_publisher.publish(self.move_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.move_forward()
    except rospy.ROSInterruptException:
        pass