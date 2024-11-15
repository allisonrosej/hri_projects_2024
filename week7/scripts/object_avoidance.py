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
        
        # Flags to control movement
        self.obstacle_detected = False
        self.person_detected_direction = None  # Stores direction towards detected person (left, right, center)
        
        # Set the rate of the loop
        self.rate = rospy.Rate(10)  # 10 Hz

    def laser_callback(self, data):
        # Process the laser scan data to detect obstacles and the person location
        front_distance = min(min(data.ranges[0:30]), min(data.ranges[330:359]))  # Front obstacles
        left_distance = min(data.ranges[30:89])  # Left side obstacles
        right_distance = min(data.ranges[270:329])  # Right side obstacles

        # Set obstacle detection flag
        self.obstacle_detected = front_distance < 0.5
        
        # Set direction towards person based on sensor data (simulate person detection)
        if right_distance < 1.5:
            self.person_detected_direction = 'right'
        elif left_distance < 1.5:
            self.person_detected_direction = 'left'
        else:
            self.person_detected_direction = 'center'

    def move_to_person(self):
        # Move the robot towards the person unless an obstacle is detected
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                # Stop moving forward and turn slightly to avoid the obstacle
                self.move_cmd.linear.x = 0.0
                if self.person_detected_direction == 'right':
                    self.move_cmd.angular.z = -0.5  # Turn right
                elif self.person_detected_direction == 'left':
                    self.move_cmd.angular.z = 0.5  # Turn left
                else:
                    self.move_cmd.angular.z = 0.3  # Turn slightly if unclear
            else:
                # Move forward and adjust direction towards the person
                self.move_cmd.linear.x = 0.2  # Move forward
                if self.person_detected_direction == 'right':
                    self.move_cmd.angular.z = -0.3  # Turn towards right
                elif self.person_detected_direction == 'left':
                    self.move_cmd.angular.z = 0.3  # Turn towards left
                else:
                    self.move_cmd.angular.z = 0.0  # Go straight

            # Publish the movement command
            self.velocity_publisher.publish(self.move_cmd)
            
            # Sleep to maintain loop rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.move_to_person()
    except rospy.ROSInterruptException:
        pass



