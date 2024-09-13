
import rospy
from std_msgs.msg import Position

def callback(data):
    rospy.loginfo("Recieved position: x=%f, y=%f, z=%f " , data.x, data.y, data.z)

def listener():

  
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('position', String, callback)

    rospy.spin()

if __name__ == '__main__':
    
    try:
    
        listener()
    except rospy.ROSInterruptException:
        
        pass
