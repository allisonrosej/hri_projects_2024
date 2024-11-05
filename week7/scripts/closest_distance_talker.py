import rospy
from sensor_msgs.msg import LaserScan# Ensure this message type is correctly defined

def talker():
    pub = rospy.Publisher('chatter', sensor_msgs, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    
       
        # Log and publish the message
        while not rospy.is_shutdown():
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

