# udacity-go-chase-it
##Project 2: Go Chase It! From Udacity Robotics Engeneer Nanodegree##

Special thanks to Forrest Edwards for Ackermann steering gazebo plugin!
https://github.com/froohoo/ackermansteer

###How to install###
Simply put all three packages to the catkin_ws/src directory and run
    catkin_make
from catkin_ws directory. This should build all the packages and Ackermann steering plugin without problems!

###How to use###
First you need to launch the world simulation with
    roslaunch my_robot world.launch
Then you need to start the robot control program with
    roslaunch ball_chaser ball_chaser.launch
And youre good to go!

Theres a white ball inside the world in gazebo. Simply put this ball infront of the robot (somewhere where robot could see it) - and it will start chasing it!


###Known Issues###
####Gazebo crash####
Sometimes gazebo just crashes - simply relaunch it.
####Robot isn't moving####
The robot's motors are controlled through PID controller. So allow the robot some time to get moving after it detects the white ball. This wind-up time can also be increased based on your GPU power, so wait up to one minute - the robot wont disappoint you :)
