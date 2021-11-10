#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <beginner_tutorials/my_service.h>

std::shared_ptr<ros::NodeHandle> nh;

TEST(MYTESTSuite, my_service_test1) {
  ros::ServiceClient client = nh->serviceClient<beginner_tutorials
  ::my_service>("my_service");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);
}

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
