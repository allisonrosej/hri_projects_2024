#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32  

class RobotController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('robot_controller', anonymous=True)
        
        # Publishers and subscribers
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.person_subscriber = rospy.Subscriber('/person_detected', Float32, self.person_callback)  # Replace Float32 with actual message type
        
        # Movement command and state variables
        self.move_cmd = Twist()
        self.obstacle_detected = False
        self.person_direction = None  # Angle in radians to the detected person
        self.rate = rospy.Rate(10)

    def laser_callback(self, data):
        """
        Callback for LaserScan data. Detects obstacles within a threshold distance.
        """
        # Filter valid ranges and find the minimum distance
        valid_ranges = [r for r in data.ranges if r != float('Inf') and r != float('NaN')]
        min_distance = min(valid_ranges, default=float('Inf'))
        self.obstacle_detected = min_distance < 0.5  # Threshold for obstacle detection

    def person_callback(self, data):
        """
        Callback for person detection. Updates the direction of the detected person.
        """
        self.person_direction = data.data  # Assuming the data is the angle in radians

    def move_toward_person(self):
        """
        Main control loop to move the robot toward the detected person while avoiding obstacles.
        """
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                # If an obstacle is detected, stop and rotate to find a clear path
                rospy.loginfo("Obstacle detected! Avoiding...")
                self.move_cmd.linear.x = 0.0
                self.move_cmd.angular.z = 0.5  # Rotate in place
            elif self.person_direction is not None:
                # If a person is detected, align and move toward them
                rospy.loginfo(f"Person detected at angle {self.person_direction:.2f} radians. Moving toward...")
                self.move_cmd.linear.x = 0.2  # Move forward at constant speed
                self.move_cmd.angular.z = -0.5 * self.person_direction  # Adjust angle
            else:
                # If no person is detected, stop the robot
                rospy.loginfo("No person detected. Stopping...")
                self.move_cmd.linear.x = 0.0
                self.move_cmd.angular.z = 0.0

            # Publish the velocity command
            self.velocity_publisher.publish(self.move_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.move_toward_person()
    except rospy.ROSInterruptException:
        pass