#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from people_msgs.msg import People
import math

# Publisher for cmd_vel
pub = None  # Initialize globally so that it can be used in move_to_goal

def move_to_goal(goal_x, goal_y):
    """
    Moves the robot to the specified goal (x, y).
    """
    cmd_vel = Twist()

    # Proportional control gains
    linear_k = 0.5
    angular_k = 2.0

    # Calculate distance and angle to the goal
    distance = math.sqrt(goal_x ** 2 + goal_y ** 2)
    angle = math.atan2(goal_y, goal_x)

    while distance > 0.1:
        # Update distance and angle
        distance = math.sqrt(goal_x ** 2 + goal_y ** 2)
        angle = math.atan2(goal_y, goal_x)

        # Set velocity commands
        cmd_vel.linear.x = linear_k * distance
        cmd_vel.angular.z = angular_k * angle

        # Publish velocity
        pub.publish(cmd_vel)

        # Sleep for control loop
        rate.sleep()

    # Stop the robot
    cmd_vel.linear.x = 0
    cmd_vel.angular.z = 0
    pub.publish(cmd_vel)

def people_callback(msg):
    """
    Callback function for the People message.
    """
    if not msg.people:
        rospy.loginfo("No people detected.")
        return

    # Check if there's a group (simplified example)
    rospy.loginfo("People detected, determining goal...")

    if len(msg.people) > 3:  # Assume more than 3 people form a group
        # Move to the center of the circle (average position)
        avg_x = sum(person.position.x for person in msg.people) / len(msg.people)
        avg_y = sum(person.position.y for person in msg.people) / len(msg.people)
        rospy.loginfo(f"Moving to the center of the group: ({avg_x}, {avg_y})")
        move_to_goal(avg_x, avg_y)
    else:
        # Move to the end of the line (farthest person)
        farthest_person = max(msg.people, key=lambda p: math.sqrt(p.position.x**2 + p.position.y**2))
        goal_x = farthest_person.position.x
        goal_y = farthest_person.position.y
        rospy.loginfo(f"Moving to the end of the line: ({goal_x}, {goal_y})")
        move_to_goal(goal_x, goal_y)

if __name__ == '__main__':
    rospy.init_node('group_navigation_node')
    rospy.loginfo("Starting group_navigation_node...")

    # Publisher for cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Subscriber for People messages
    rospy.Subscriber('/people', People, people_callback)

    rate = rospy.Rate(10)  # 10 Hz control loop

    rospy.spin()
