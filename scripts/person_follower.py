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

    def linear_vel(self, x):
        """ Determine the forward velocity based on distance away
        from the object """

        if x >= dist:
            # keep going if robot is far away from the object
            return 0.2
        else:
            # stop if robot reaches next to the object
            return 0

    def angular_vel(self, theta):
        """Determine the angular velocity based on angle to the object """

        if abs(theta) < 10:
            # limit angular velocity with a buffer to prevent the 
            #   "over-jiggliness" of the robot when it stops in front 
            #   of the object
            return 0
        else:
            # prevent > 180 degree turns, robot will only make <= 180 
            #   degree turns to face the object
            if theta < 180:
                return theta
            else:
                return theta - 360

    def process_scan(self, data):
        """ Take in the scan data and determine how much the robot
        should move and turn to follow the object at a safe distance """
        
        # get the minimum distance and its angle for each scan
        min_data = self.find_min(data)
        min_dist = min_data[0]
        min_ang = min_data[1]

        if min_dist == math.inf:
            # handle case when no objects are presented
            self.twist.linear.x = 0
            self.twist.angular.z = 0
        else:
            # set linear and angular velocities
            self.twist.linear.x = self.linear_vel(min_dist)
            self.twist.angular.z = math.radians(self.angular_vel(min_ang))

        # publish Twist msg
        self.twist_pub.publish(self.twist)

    def run(self):
        """ Run the node """

        # keep the program alive
        rospy.spin()

if __name__ == '__main__':
    # declare a node and run it
    node = PersonFollower()
    node.run()