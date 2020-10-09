# Wall-follower-in-ROS-using-Python

This was my final project for a robotics course in UMD. (ENPM809E - Python Applications for Robotics)

## Task
The goal of this project is to solve a maze problem using only one Turtlebot 3 robot which comes equipped with a laser scanner.

We had to achieve the following tasks:

* **Detect a wall
* **Follow the wall
* **Avoid collision with obstacles
* **Find the exit

We had to accomplish the Tasks automatically.

To execute the same, I created one python package(FinalProject_ws) which has one module (motion_plan).

### Follow_wall.py: 

This program consists of various fiunctions to check if the bot has detected a wall and based on the distance from obstacle on 
right side and left side, the program will either enter the algorithm for Right wall follower or the Left wall follower.		



## Instructions to run the code:(In PyCharm)

* **Download the 'FinalProject_Nimbekar.zip' folder from Canvas and unzip it.

* **Run the `roscore` command in terminal to start ros.

* **Run the maze in one terminal:
```
roslaunch maze maze.launch
```
* **Then run the Code in another terminal: 
```
rosrun motion_plan follow_wall.py
```
* **Manually stop the code once the maze is cleared.

