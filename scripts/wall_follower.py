#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import math

# define distance to wall
dist = 0.8

class WallFollower(object):
    """ This node makes the robot drive alongside the walls of a square
    room indefinitely """

    def __init__(self):
        """ Initial setups for the node """

        # set up node, subscriber, publisher, and a zeroed Twist msg
        rospy.init_node("wall_follower")
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.twist = Twist()

    def process_scan(self, data):
        """WHAT THIS FUNCTION DOES"""
        front_dist = data.ranges[0]
        front_right_dist = data.ranges[314]
        right_dist = data.ranges[269]
        back_right_dist = data.ranges[224]
        kp_lin = 0.2
        kp_ang = 10
        
        limit_vel = lambda v: 0.3 if v > 0.3 else v
        angular_vel = lambda x: kp_ang * (dist - front_dist) if x < dist else (back_right_dist - front_right_dist) + (dist - right_dist) 

        self.twist.linear.x = limit_vel(kp * front_dist)
        self.twist.angular.z = angular_vel(front_dist)
        self.twist_pub.publish(self.twist)

    def run(self):
        """ Run the node """

        # keep the program alive
        rospy.spin()

if __name__ == '__main__':
    # declare a node and run it
    node = WallFollower()
    node.run()