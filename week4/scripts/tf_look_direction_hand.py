#!/usr/bin/python3
import rospy
import math
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Float64

def look_in_hand_direction():
    rospy.init_node('look_in_hand_direction')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    head_pub = rospy.Publisher('/head_controller/command', Float64, queue_size=10)
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('LFinger23_link', 'Hand', rospy.Time())
            # Calculate the angle to look in the direction the hand is pointing
            angle = math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            head_pub.publish(angle)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        rate.sleep()

if __name__ == '__main__':
    look_in_hand_direction()