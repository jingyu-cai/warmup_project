## Driving in a Square
### A high-level description
The problem is to make the robot drive in a square, and for me, I tried to make the robot go forwards for a specific amount of time, and turn for a specific amount of time so that it turns 90 degrees.
### Code explanation
* `__init__`: I initialized the node and the publisher, defined an zeroed Twist message to be modified later, as well as made the robot sleep for 1 second for the publisher to get set up. 
* `run()`: When the robot is not shut down, I looped the publisher to publish the two different Twist messages along with `rospy.sleep()` so that the robot will go forwards for 10 seconds and turn for 5 seconds, and I specifically chose the `linear.x` velocity to be 0.1 for the forward velocity so the robot won't drift as much and the `angular.z` velocity to be `radians(18)` for the turn velocity so that the robot can turn 90 degrees in 5 seconds. 
### A gif
![drive_square_demo.gif](gifs/drive_square_demo.gif)
