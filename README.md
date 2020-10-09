# Wall-follower-in-ROS-using-Python

This was my final project for a robotics course in UMD. (ENPM809E - Python Applications for Robotics)

## Task
The goal of this project is to solve a maze problem using only one Turtlebot 3 robot which comes equipped with a laser scanner.

We had to achieve the following tasks:

* Detect a wall
* Follow the wall
* Avoid collision with obstacles
* Find the exit

We had to accomplish the Tasks automatically.

### Test Environment
- ROS: Melodic
- Ubuntu: 18.04LTS
- Python: 3.6.9
- (Gazebo: 9.0.0)

### Approach

To execute the same, I created one python package(FinalProject_ws) which has one module (motion_plan).

### Follow_wall.py: 

This program consists of various fiunctions to check if the bot has detected a wall and based on the distance from obstacle on 
right side and left side, the program will either enter the algorithm for Right wall follower or the Left wall follower.		



## Instructions to run the code:(In PyCharm)

* Download the 'FinalProject_Nimbekar.zip' folder from Canvas and unzip it.

* Run the `roscore` command in terminal to start ros.

* Run the maze in terminal one:
```
roslaunch maze maze.launch
```
* Then run the Code in terminal two: 
```
rosrun motion_plan follow_wall.py
```
* Manually stop the code once the maze is cleared.

## Built With

* [ROS](http://www.ros.org/) - Set of software libraries and tools used to build the robot
* [Gazebo](http://gazebosim.org/) - Tool used to simulation
* [Python](https://www.python.org/) - Programming language

## Author
[Nupur Nimbekar](https://github.com/nimbekarnd)
