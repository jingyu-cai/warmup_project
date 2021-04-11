#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

# define distance to wall
dist = 1.0

# define kp for linear velocity
kp_lin = 0.2

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
        """ Sets the linear velocity of the robot with proportional control """

        # define error signal as the distance to the wall in front of the robot
        front_dist = data.ranges[0]

        # calculates linear velocity with kp_lin
        vel = kp_lin * front_dist

        if vel > 0.3:
            # limits the maximum velocity in case the robot drifts immediately
            #   after turning, which can cause it to spin in circles
            vel = 0.3
        
        return vel

    def far_from_wall(self, data):
        """ Checks if the robot is far away from any of the walls """

        for x in data.ranges:
            if x < dist * 1.5:
                # sometimes the robot may cross the dist threshold during 
                #   moving and turning along the walls, so * 1.5 is added 
                #   to provide a buffer
                return False
        
        return True
    
    def angular_vel(self, data):
        """ Sets the angular velocity of the robot with proportional control """

        # define dists as error signals or to be used to compute error signals
        front_dist = data.ranges[0]
        front_right_dist = data.ranges[314]
        right_dist = data.ranges[269]
        back_right_dist = data.ranges[224]

        if self.far_from_wall(data):
            # makes the robot go straight until it's close to a wall
            return 0
        else:
            # when the robot is close to a wall enough that it can detect 
            #   the wall at either its front right or back right or both

            # use the back_right_dist (dist to wall at 225 degrees) minus
            #   front_right_dist (dist to wall at 315 degrees) as error signal,
            #   it will balance the robot so it moves in a straight line and 
            #   turn at corners
            balance_ang = back_right_dist - front_right_dist

            # use the defined dist minus right_dist (dist to wall at 270 
            #   degrees) as error signal, it will make the robot keep a 
            #   distance to the wall, which would be on its right since I 
            #   am making the robot drive counterclockwise
            dist_ang = dist - right_dist


            if balance_ang == math.inf:
                # handle the case when robot cannot detect anything on its
                #   back right but can detect the wall on its front right,
                #   it will turn counterclockwise until it detects the wall 
                #   at both angles
                return front_right_dist
            elif balance_ang == -1 * math.inf:
                # handle the case when robot cannot detect anything on its
                #   front right but can detect the wall on its back right,
                #   it will turn clockwise until it detects the wall at 
                #   both angles
                return -1 * back_right_dist
            else:
                # when the robot detects the wall at both angles, it will use
                #   both error signals to drive alongside the walls and turn
                #   at corners
                return balance_ang + dist_ang

    def process_scan(self, data):
        """ WHAT THIS FUNCTION DOES """

        # set linear and angular velocities, and publish Twist msg
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