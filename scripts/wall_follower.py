#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

# define distance to wall
dist = 0.8

# define kp for linear and angular velocities
kp_lin = 0.2
kp_ang = 10

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

    def linear_vel(self, data):
        """ WHAT THIS FUNCTION DOES """

        front_dist = data.ranges[0]
        vel = kp_lin * front_dist

        if vel > 0.3:
            vel = 0.3
        
        return vel

    def far_from_wall(self, data):
        """ WHAT THIS FUNCTION DOES """

        for x in data.ranges:
            if x < dist:
                return False
        
        return True
    
    def angular_vel(self, data):
        """ WHAT THIS FUNCTION DOES """

        front_dist = data.ranges[0]
        front_right_dist = data.ranges[314]
        right_dist = data.ranges[269]
        back_right_dist = data.ranges[224]

        if self.far_from_wall(data):
            return 0
        else:
            if front_dist < dist:
                return kp_ang * (dist - front_dist)
            else:
                return (back_right_dist - front_right_dist) + (dist - right_dist) 

    def process_scan(self, data):
        """ WHAT THIS FUNCTION DOES """

        self.twist.linear.x = self.linear_vel(data)
        self.twist.angular.z = self.angular_vel(data)
        self.twist_pub.publish(self.twist)

    def run(self):
        """ Run the node """

        # keep the program alive
        rospy.spin()

if __name__ == '__main__':
    # declare a node and run it
    node = WallFollower()
    node.run()