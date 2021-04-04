#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# How close we will get to wall.
distance = 0.4

class StopAtWall(object):
    """ This node makes the robot follow its closest object while maintaining a safe distance """

    def __init__(self):
        # start the rospy node
        rospy.init_node("person_follower")

        
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

    def process_scan(self, data):
        # Determine closeness to wall by looking at scan data from in front of
        #   the robot, set linear velocity based on that information, and
        #   publish to cmd_vel.

        # The ranges field is a list of 360 number where each number
        #   corresponds to the distance to the closest obstacle from the
        #   LiDAR at various angles. Each measurement is 1 degree apart.

        # The first entry in the ranges list corresponds with what's directly
        #   in front of the robot.

        if data.ranges[0] >= distance:
            # Go forward if not close enough to wall.
            self.twist.linear.x = 0.1
        else:
            # Close enough to wall, stop.
            self.twist.linear.x = 0

        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)


    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = StopAtWall()
    node.run()