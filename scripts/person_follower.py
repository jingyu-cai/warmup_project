#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# define distance to closest object
distance = 0.5

class StopAtWall(object):
    """ This node makes the robot follow its closest object 
    while maintaining a safe distance """

    def __init__(self):
        """ Initial setups for the node """

        # set up node, subscriber, publisher, and a zeroed Twist msg
        rospy.init_node("person_follower")
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.twist = Twist()

    def process_scan(self, data):
        """ WHAT THIS FUNCTION DOES """

        min_dist = min(data)
        min_dist_i = data.index(min_dist)

        if min_dist_i != 0:
            self.twist.linear.x = 0.0
            self.twist.angular.z = radians(min_dist_i)
            self
            rospy.sleep(1)
        


        if data.ranges[0] >= distance:
            # Go forward if not close enough to wall.
            self.twist.linear.x = 0.1
        else:
            # Close enough to wall, stop.
            self.twist.linear.x = 0

        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)


    def run(self):
        """ run the node """

        # keep the program alive
        rospy.spin()

if __name__ == '__main__':
    # declare a node and run it
    node = StopAtWall()
    node.run()