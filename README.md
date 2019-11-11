[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)


# beginner_tutorials

## Overview

This repository has ROS subscriber and publisher nodes written in c++. These nodes are provided in ROS tutorials. publisher is talker.cpp, subscriber is listener.cpp, topic is "chatter". talker.cpp and listener.cpp are ros nodes. This repository also provides overview of using rosservices, roslaunch files. A ros service is created in talker.cpp file that takes a string from user and updates the message. A launch file is created that launches both talker and listener nodes simultaneously, can also set user defined frequency for communicattion between nodes.

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

## Rosbag
To record using rosbag for 15sec use the following command\
rosbag record --duration=15s /chatter\
To play the recorded bag file\
cd ~/catkin_ws\
roscore in a terminal\
rosrun beginner_tutorials listener

In another terminal\
cd ~/catkin_ws/src/beginner_tutorials/results\
rosbag play allTopicsRecord.bag

## TF frames
Talker.cpp is modified to add a broadcaster that broadcasts the transformation between /talk frame with respect\ 
to /world frame. The transformation between these two frames is broadcasted as a transform.

Run this first

cd ~/catkin_ws\
roscore in a terminal\
rosrun beginner_tutorials talker

To view frames sent 

rosrun tf view_frames  in new terminal\
evince frames.pdf to see the frames

Echo values in terminal

rosrun tf tf_echo /world /talk

To view rqt_tree\
rosrun rqt_tf_tree rqt_tf_tree

## Cppcheck

use this command for cppcheck:\
cppcheck --enable=all --std=c++11 -I ../../devel/include/ -I ../../../../../../../opt/ros/kinetic/include/ -I ../../../../../../../usr/include/ --check-config --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )

