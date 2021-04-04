#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from math import radians

class DriveSquare(object):
    """ This node publishes the robot's velocity using timing to make it 
    drive in a square """

    def __init__(self):
        """ Initial setups for the node """

        # set up node, publisher, and a zeroed Twist msg
        rospy.init_node('drive_square')
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

        # sleep for 1 second to ensure publisher is set up
        rospy.sleep(1)

    def run(self):
        """ Loop through publishing different linear and 
        angular velocities to make robot drive in a square """

        while not rospy.is_shutdown():
            # define velocity for robot to move forwards
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.0
            self.twist_pub.publish(self.twist)

            # loop publishing forward velocity for 10s
            rospy.sleep(10)

            # define velocity for robot to turn
            self.twist.linear.x = 0.0
            self.twist.angular.z = radians(18)
            self.twist_pub.publish(self.twist)

            # loop publishing turn velocity for 5s
            rospy.sleep(5)

if __name__ == '__main__':
    # declare a node and run it
    node = DriveSquare()
    node.run()