import rospy
from sensor_msgs.msg import LaserScan  # Import the LaserScan message type

def talker():
    # Create a publisher object for the 'chatter' topic, using LaserScan message type
    pub = rospy.Publisher('chatter', LaserScan, queue_size=10)

    # Initialize the node
    rospy.init_node('talker', anonymous=True)

    # Set the loop rate
    rate = rospy.Rate(10)  # 10hz

    # Create a LaserScan message
    message = LaserScan()
    
    # Fill the message with appropriate data (here, I'll just set dummy values)
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = "laser_frame"
    message.angle_min = -3.14  # Example values
    message.angle_max = 3.14
    message.angle_increment = 0.01
    message.time_increment = 0.0
    message.scan_time = 0.1
    message.range_min = 0.0
    message.range_max = 10.0
    message.ranges = [5.0] * 360  # Example: a flat scan with distance of 5m at every angle
    message.intensities = [100.0] * 360  # Example intensities
    
    # Log and publish the message
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing LaserScan message")
        pub.publish(message)  # Publish the message
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

