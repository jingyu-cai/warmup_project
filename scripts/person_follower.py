#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import math

# define distance to closest object
dist = 0.5

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

    def find_min(self, data):
        """ WHAT THIS FUNCTION DOES """

        if len(data.ranges) < 1:
            print("find_min(): data has no elements")
        else:
            min_element = data.ranges[0]
            min_index = 0
            for i in range(len(data.ranges)):
                if data.ranges[i] < min_element:
                    min_element = data.ranges[i]
                    min_index = i
        return (min_element, min_index)

    def process_scan(self, data):
        """ WHAT THIS FUNCTION DOES """
        
        min_data = self.find_min(data)

        angle = lambda theta: theta if theta < 180 else theta - 360
        turn = lambda theta: 0 if abs(theta) < 10 else theta

        if min_data[0] >= dist:
            self.twist.linear.x = 0.2
        else:
            self.twist.linear.x = 0.0

        self.twist.angular.z = math.radians(turn(angle(min_data[1])))

        self.twist_pub.publish(self.twist)

    def run(self):
        """ run the node """

        # keep the program alive
        rospy.spin()

if __name__ == '__main__':
    # declare a node and run it
    node = StopAtWall()
    node.run()