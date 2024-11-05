#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    # Find the closest distance in the laser scan ranges
    closest_distance = min(msg.ranges)
    rospy.loginfo(f"Closest distance: {closest_distance} meters")

def main():
    # Initialize the ROS node
    rospy.init_node('closest_distance_listener', anonymous=True)
    
    # Subscribe to the /base_scan topic
    rospy.Subscriber('/base_scan', LaserScan, scan_callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()

      
