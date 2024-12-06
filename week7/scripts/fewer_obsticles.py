import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoider:
    def __init__(self):
        rospy.init_node('obstacle_avoider', anonymous=True)
        
        # Publisher to control robot's velocity
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscriber to get LIDAR scan data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Initialize Twist message for controlling movement
        self.move_cmd = Twist()
        
        # Safe distance threshold (in meters)
        self.safe_distance = 0.5
        
        rospy.spin()

    def scan_callback(self, scan_data):
        # Get distances in front, left, and right directions
        front_distance = min(min(scan_data.ranges[0:10]), min(scan_data.ranges[-10:]))
        left_distance = min(scan_data.ranges[80:100])
        right_distance = min(scan_data.ranges[260:280])

        if front_distance < self.safe_distance:
            # Obstacle detected in front, stop moving forward
            self.move_cmd.linear.x = 0.0
            
            # Decide which direction has fewer obstacles
            if left_distance > right_distance:
                # Turn left if left side is clearer
                self.move_cmd.angular.z = 0.5
            else:
                # Turn right if right side is clearer
                self.move_cmd.angular.z = -0.5
        
        else:
            # No obstacle in front, move forward
            self.move_cmd.linear.x = 0.5
            self.move_cmd.angular.z = 0.0
        
        # Publish movement command
        self.cmd_vel_pub.publish(self.move_cmd)

if __name__ == '__main__':
    try:
        ObstacleAvoider()
    except rospy.ROSInterruptException:
        pass