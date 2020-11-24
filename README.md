# Johnnie_The_Turtlebot_Walker
A software that implements a simple obstacle avoidance in a turtlebot. The robot uses laser scan to find obstacles. If the minimum of the laser scan drops below a threshold, then it is near an object. The controller commands the robot to rotate away from the obstacle

## Dependencies
This repositories requires Ubuntu 18, ROS Melodic, Catkin, Gazebo and Turtlebot simulator.

To install ROS, refer http://wiki.ros.org/melodic/Installation/Ubuntu
To install Turtlebot simulator, refer https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/
## Download and Installation
In a terminal run the following
```
$ mkdir -p <workspace>/src
$ cd <workspace>/src
$ git clone https://github.com/clueless-bachu/Johnnie_The_Turtlebot_Walker.git
$ cd ../..
$ catkin_make
$ source devel/setup.bash
```

## Running the simulator
To run the obstacle avoidance launcg file, run the following in a terminal
```
$ roslaunch johnnie_the_turtlebot_walker johnie_turtlebot.launch record:=<true or false>
```

You can add more objects!

## playing the ROS bag data 
To playback the recorded rosbag data, run the following in a new terminal
```
cd <workspace>/src/johnnie_the_turtlebot_walker/results
rosbag info rec.bag 
```