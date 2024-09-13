
import rospy
from std_msgs.msg import Position

def talker():
    pub = rospy.Publisher('chatter', 'position', Position, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
       pos = Position()
       pos.x = 1.0
       pos.y = 2.0
       pos.z = 3.0
       rospy.loginfo("Publishing: %s" , pos)
       pub.publish(pos)
       rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
