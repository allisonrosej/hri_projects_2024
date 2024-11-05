import rospy
from geometry_msgs.msg import Twist
import math
import time

# Function to move the robot in a figure-eight pattern
def move_figure_eight():
    rospy.init_node('move_figure_eight', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1) 
    move_cmd = Twist()

    for _ in range(2):  # Two loops of figure eight
        # First loop (left curve)
        move_cmd.linear.x = 0.5
        move_cmd.angular.z = math.radians(45)  # Turn left gradually
        pub.publish(move_cmd)
        time.sleep(4)  # Move for 4 seconds
        
        # Second loop (right curve)
        move_cmd.angular.z = -math.radians(45)  # Turn right gradually
        pub.publish(move_cmd)
        time.sleep(4) 

    # Stop the robot after completing figure-eight
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    
    

