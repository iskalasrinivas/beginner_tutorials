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
 * @file    talkertest.cpp
 * @author  Raja Srinivas Iskala
 * @copyright BSD 3-clause
 * @brief This main file for tests which runs all the tests of talker node.
 */

#include "ros/ros.h"
#include "gtest/gtest.h"
#include "ros/service_client.h"
#include "beginner_tutorials/changeBaseString.h"

TEST(testTalkerNode, testServiceExistence) {
  // node handle
  ros::NodeHandle nodeHandler;
  // calling service client method
  auto client = nodeHandler.serviceClient < beginner_tutorials::changeBaseString
      > ("changeBaseString");
  // checking if client exists
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
}

TEST(testTalkerNode, testServiceMessageUpdate) {
  // node handle
  ros::NodeHandle nodeHandler;

  // Register a client to the service
  auto client = nodeHandler.serviceClient < beginner_tutorials::changeBaseString
      > ("changeBaseString");
  // create service object
  beginner_tutorials::changeBaseString serve;

  // request for change in base string
  serve.request.baseString = "hello!!";

  // call the service
  client.call(serve.request, serve.response);

  // check if the output string matches with the base string
  EXPECT_STREQ("hello!!", serve.response.changedBaseString.c_str());
}

