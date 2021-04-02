## Driving in a Square
### A high-level description
The problem is to make the robot drive in a square, and for me, I tried to make the robot go forwards for a specific amount of time, and turn for a specific amount of time so that it turns 90 degrees.
### Code explanation
I have two main functions under my class - the `__init__` method and the `run()` method - to initialize the node and necessary variables and run the node to publish the messages. Lastly, I instantiated and ran the node within `if __name__ == '__main__':`.
* `__init__`: I initialized the node and the publisher, as well as made the robot sleep for 5 seconds for the publisher to get set up. 
* `run()`: I defined the forward and turning velocities, and when the robot is not shut down, I looped the publisher so that the robot will go forwards for 10 seconds and turn for 5 seconds, and I specifically chose the angular.z velocity to be radians(18) so that the robot can turn 90 degrees in 5 seconds. 
### A gif
![drive_square_demo.gif](gifs/drive_square_demo.gif)