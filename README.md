## Driving in a Square
### A high-level description
The problem is to make the robot drive in a square, and for me, I tried to make the robot go forwards for a specific amount of time, and turn for a specific amount of time so that it turns 90 degrees.
### Code explanation
* `__init__`: I initialized the node and the publisher, defined an zeroed Twist message to be modified later, as well as made the ROS sleep for 1 second for the publisher to get set up. 
* `run()`: When the robot is not shut down, I looped the publisher to publish the two different Twist messages along with `rospy.sleep()` so that the robot will go forwards for 10 seconds and turn for 5 seconds. I specifically chose the `linear.x` velocity to be 0.1 for the forward velocity so the robot won't drift as much and the `angular.z` velocity to be `radians(18)` for the turn velocity so that the robot can turn 90 degrees in 5 seconds. 
### A gif
![drive_square_demo.gif](gifs/drive_square_demo.gif)

## Challenges
For the **Driving in a Square** behavior, my biggest challenge was understanding how to use `angular.z` to turn the robot 90 degrees. For that, I read on documentations of `angular.z` and found its units to be rad/sec, so I was able to set a timer to leave the robot turning at a specific angular velocity for a specific amount of time to make it turn 90 degrees with simple math.

## Future work
For the **Driving in a Square** behavior, the biggest improvement would be to make the robot walk in a perfect, or near-perfect, square everytime. One possible way to solve that is to set a baseline before the start of every forward and turning movement, so that if the angles deviate above a certain threshold, the robot can correct itself by turning itself back onto the correct trajectory.

## Takeaways
* My first takeaway is that the movement of the TurtleBot does not perfectly reflect what my code tells it to do. Prior to this course, coding something meant that the system will execute my commands flawlessly. But the TurtleBot simulates real-world environments, so due to elements like friction and imprecise control of physical motors, the commands won't always be executed exactly as expected. Thus, we should always keep an open mind for a range of possible outcomes and adjust the code to simulate our desired behavior as best as we can.
