import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def publish_joint_states(pub, joint_names, positions):
    js = JointState()
    js.header = Header()
    js.header.stamp = rospy.get_rostime()
    js.header.frame_id = "Torso"
    js.name = joint_names
    js.position = positions
    pub.publish(js)
    rospy.loginfo(js)

def wave_hand(pub, joint_names):
    for i in range(3):
        positions = [0, 0, 0, math.radians(45), 0, math.radians(60)]
        publish_joint_states(pub, joint_names, positions)
        rospy.sleep(0.5)

        positions = [0, 0, 0, math.radians(-45), 0, math.radians(-60)]
        publish_joint_states(pub, joint_names, positions)
        rospy.sleep(0.5)


def nod_head(pub, joint_names):
    for i in range(2):
        positions = [math.radians(0), math.radians(30), 0, 0, 0, 0]
        publish_joint_states(pub, joint_names, positions)
        rospy.sleep(0.5)

        positions = [math.radians(0), math.radians(-30), 0, 0, 0, 0]
        publish_joint_states(pub, joint_names, positions)
        rospy.sleep(0.5)

def shake_head(pub, joint_names):
    for i in range(2):
        # Look right
        positions = [math.radians(30), 0, 0, 0, 0, 0]
        publish_joint_states(pub, joint_names, positions)
        rospy.sleep(0.5)

        positions = [math.radians(-30), 0, 0, 0, 0, 0]
        publish_joint_states(pub, joint_names, positions)
        rospy.sleep(0.5)

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    joint_names = ["HeadYaw", "HeadPitch", "RightShoulderPitch", "RightShoulderRoll", "RightElbowYaw", "RightElbowRoll"]

    wave_hand(pub, joint_names)
    rospy.sleep(1)  # Delay between actions

    nod_head(pub, joint_names)
    rospy.sleep(1)

    shake_head(pub, joint_names)
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
