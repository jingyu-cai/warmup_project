#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from math import radians

class DriveSquare(object):
    """ This node publishes the robot's velocity using timing to make it drive in a square """

    # set up the node and create publisher
    def __init__(self):
        rospy.init_node('drive_square')
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.sleep(5)

    # run the node
    def run(self):
        # define velocity when robot is moving forwards
        forward_velocity = Twist()
        forward_velocity.linear.x = 0.1
        forward_velocity.angular.z = 0.0

        # define velocity when robot is turning
        turn_velocity = Twist()
        turn_velocity.linear.x = 0.0
        turn_velocity.angular.z = radians(18)

        # loop publishing forward velocity for 10s, and turn velocity for 5s
        while not rospy.is_shutdown():
            self.velocity_pub.publish(forward_velocity)
            rospy.sleep(10)
            self.velocity_pub.publish(turn_velocity)
            rospy.sleep(5)

# main method
if __name__ == '__main__':
    node = DriveSquare()
    node.run()