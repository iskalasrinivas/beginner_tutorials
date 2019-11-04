[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)


# beginner_tutorials

## Overview

This repository has ROS subscriber and publisher nodes written in c++. These nodes are provided in ROS tutorials. publisher is talker.cpp, subscriber is listener.cpp, topic is "chatter". talker.cpp and listener.cpp are ros nodes.

## Dependencies
Distro: ROS Kinetic
The package depends on cpp_common, rostime, roscpp_traits, roscpp_serialization, catkin, genmsg, genpy, message_runtime
gencpp, geneus, gennodejs, genlisp, message_generation, rosbuild, rosconsole, std_msgs, rosgraph_msgs, xmlrpcpp
roscpp, rosgraph, ros_environment, rospack, roslib, rospy.


## build instructions in terminal

source /opt/ros/kinetic/setup.bash\
mkdir -p ~/catkin_ws/src\
cd catkin_ws\
source devel/setup.bash\
cd src\
git clone https://github.com/iskalasrinivas/beginner_tutorials.git \
cd ..\
catkin_make 

## Run instructions

The above commands creates executable for listener and talker. open terminal

create master with roscore.

in new terminal
rosrun beginner_tutorials talker\
rosrun beginner_tutorials listener

## Using ROS Service

Make a service call to the service /changeBaseString\
rosservice call /changeBaseString "pass the new message in quotes."\
The nodes has to be running to use the service.

## ROS Launch

We can launch both nodes simultaneously by using roslaunch beginner_tutorials week10HW.launch\
roslaunch begineer_tutorials week10HW.launch frequency:= 10\
usage roslaunch [package name][launch file name]\
This also launches the master node in case master not running.

## RQT_Console and RQT_logger
to run RQT_ console: In a new terminal enter the command rqt_console .\
to run rqt_logger : run rosrun rqt_logger_level rqt_logger_level .


