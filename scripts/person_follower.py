#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import math

# define distance to closest object
dist = 0.5

class PersonFollower(object):
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
        """ Find the minimum element and its index in the ranges tuple,
        which correspond to the distance to the nearest object from the
        robot and the angle it points to """

        data_len = len(data.ranges)

        if data_len < 1:
            # handle case when data has no elements
            print("find_min(): data has no elements")
        else:
            # find the minimum element and its index
            min_element = data.ranges[0]
            min_index = 0
            for i in range(data_len):
                if data.ranges[i] < min_element:
                    min_element = data.ranges[i]
                    min_index = i
        return (min_element, min_index)

    def process_scan(self, data):
        """ Take in the scan data and determine how much the robot
        should move and turn to follow the object at a safe distance """
        
        # get the minimum distance and its angle for each scan
        min_data = self.find_min(data)

        if min_data[0] == math.inf:
            # handle case when no objects are presented
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.twist_pub.publish(self.twist)
        else:
            # lambda function to determine forward velocity based on distance
            #   away from the object
            linear_vel = lambda x: 0.2 if min_data[0] >= dist else 0

            # lambda function to prevent > 180 degree turns, robot will only 
            #   make <= 180 degree turns
            limit_angle = lambda theta: theta if theta < 180 else theta - 360

            # lambda function to determine turn velocity with a buffer to 
            #   prevent the "over-jiggliness" of the robot when it stops in 
            #   front of the object
            turn_vel = lambda theta: 0 if abs(theta) < 10 else theta

            # set linear and angular velocities and publish Twist msg
            self.twist.linear.x = linear_vel(min_data[0])
            self.twist.angular.z = math.radians(turn_vel(limit_angle(min_data[1])))
            self.twist_pub.publish(self.twist)

    def run(self):
        """ Run the node """

        # keep the program alive
        rospy.spin()

if __name__ == '__main__':
    # declare a node and run it
    node = PersonFollower()
    node.run()