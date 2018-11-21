# Turtlebot - Walker - ROS turtlebot project for simple automated driving | ENPM 808X
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---

## Overview

An implementation of ROS turtlebot package for a simple autonomous driving with collision avoidance:

## Dependencies

* The implementation setup requires ROS kinetic, catkin, ROS turtlebot package, Gazebo running on Ubuntu 16.04 distribution
* Follow the [tutorial](http://wiki.ros.org/kinetic/Installation/Ubuntu) to install ROS
* Follow the [link](https://catkin-tools.readthedocs.io/en/latest/installing.html) to install catkin
* To install turtlebot dependencies:
```
sudo apt-get install ros-kinetic-turtlebot-gazebo 
ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
``` 

## Building the repo via command-line
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/vijay4313/turtlebot_rhoomba.git
cd ..
catkin_make
```

## Running Demo
#### To run with launch file
```
cd ~catkin_ws
source ./devel/setup.bash
roslaunch turtlebot_rhoomba turtlebot_rhoomba.launch
```
The launcher will launch a gazebo interface with turtlebot in motion.
to exit the simulation, in the terminal, press Ctrl+C

#### To run without launch file
In a new terminal type:
```
roscore
```
In another terminal, launch the turtlebot gazebo world
```
cd ~catkin_ws
source ./devel/setup.bash
roslaunch turtlebot_gazebo turtlebot_world.launch
```
In another terminal, launch the turtlebot_rhoomba package
```
cd ~/catkin_ws
source devel/setup.bash
rosrun turtlebot_rhoomba turtlebot_rhoomba_node
```
To stop the simulation, press Ctrl+C on all active terminal

## Recording & Playing ROS messages
* To Record all the topics (except the camera topics) from turtlebot from launch file:
```
cd ~catkin_ws
source ./devel/setup.bash
roslaunch  turtlebot_gazebo turtlebot_world.launch record:=1
```
The recorded bag file is saved in ~/catkin_ws/src/turtlebot_rhoomba/results/ as turtlebot_record.bag

* Playing the recorded messages
```
rosbag play <path to turtlebot_record.bag file>
```
Note: For rosbag playback, Gazebo shouldn't be working.
To stop the playback, press Ctrl+C on the terminal.
