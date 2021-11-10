/**
 * MIT License
 *
 * Copyright (c) 2021 Maaruf Vazifdar
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to 
 * deal in the Software without restriction, including without limitation the  
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE. 
 * 
 * @file talker_test.cpp
 * @author Maaruf Vazifdar
 * @brief Test code for talker node.
 * @version 1.0
 * @date 11/07/2021
 * @copyright  Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <beginner_tutorials/my_service.h>

std::shared_ptr<ros::NodeHandle> nh;

// Test to check if the service started and is active.
TEST(MYTESTSuite, my_service_test1) {
  ros::ServiceClient client = nh->serviceClient<beginner_tutorials
  ::my_service>("my_service");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);
}

// Test to check if service retrurns string correctly.
TEST(MYTESTSuite, my_service_test2) {
  ros::ServiceClient client = nh->serviceClient<beginner_tutorials
  ::my_service>("my_service");
  beginner_tutorials::my_service srv;
  srv.request.input = "test string";
  client.call(srv);

  EXPECT_EQ("test string", srv.response.output);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker_test_node");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
