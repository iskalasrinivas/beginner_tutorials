/**
 * Copyright (c) 2019, RAJA SRINIVAS ISKALA
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 **/

/**
 * @file talker.cpp
 * @author Raja Srinivas Iskala
 * @copyright 2019
 * @copyright BSD 3-Clause
 * @date 04/11/2019
 * @brief This code is a Ros publisher node that publishes a string message
 * on topic called chatter taken from ROS tutorials page.
 */

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
#include "beginner_tutorials/changeBaseString.h"
#include "talk/talker.h"

/**
 * object of DefaultMessage struct
 */
DefaultMessage message;

/**
 * @brief  callback function for the Service changeBaseString
 * @param  requestService   The request that is sent to the service
 * @param  responseClient   The response given by the service to the client
 * @return bool
 */
bool changeString(
    beginner_tutorials::changeBaseString::Request &requestService,
    beginner_tutorials::changeBaseString::Response &responseClient) {
  message.defaultMessage = requestService.baseString;
  ROS_WARN_STREAM("Client has modified the default text to :");
  responseClient.changedBaseString = requestService.baseString;
  return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

/**
 * @brief the main function of the talker file
 * @param argc  Specifies the number of elements within argv
 * @param argv  Array of c-string pointers
 * @return 0 int
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // create a broadcaster object
  static tf::TransformBroadcaster broadcaster;

  // create a tf transform object
  tf::Transform transform;

  // declaring variable to denote loop frequency to 5 Hz default.
  int rate = 5;

  // converts string argument to int
  if (argc > 1) {
    rate = atoi(argv[1]);
  }

  if (rate > 0) {
    ROS_DEBUG_STREAM("The loop is operating at frequency of Hz: " << rate);
  } else if (rate < 0) {
    ROS_ERROR_STREAM("The input frequency cannot be less than Zero");

    ROS_WARN_STREAM("Correction: Defaulting frequency back to 5 Hz");

    // Reset loopRate back to default (5 Hz)
    rate = 5;
  } else if (rate == 0) {
    ROS_FATAL_STREAM("The input frequency cannot be Zero");

    ROS_WARN_STREAM("Correction: Defaulting frequency back to 5 Hz");

    // Reset loopRate back to default (5 Hz)
    rate = 5;
  }

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  auto chatter_pub = n.advertise < std_msgs::String > ("chatter", 1000);

  // calling advertiseService function with function callback changeString
  auto server = n.advertiseService("changeBaseString", changeString);

  ros::Rate loop_rate(rate);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << message.defaultMessage;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    // set rotation using roll pitch yaw
    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, 1);
    transform.setRotation(quaternion);

    // setting translation
    transform.setOrigin(tf::Vector3(5.0, 5.0, 0.0));

    // broadcast this transform
    broadcaster.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

