#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/subscribe_options.h>

bool message_received = false;

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  if (msg->data == "Hello World 0") {
    message_received = true;
  }
}

TEST(TestListener, testMessageReception) {
  int argc = 0;
  char **argv = nullptr;
  ros::init(argc, argv, "test_listener");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  // Simulate message publishing
  std_msgs::String msg;
  msg.data = "Hello World 0";
  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);
  pub.publish(msg);

  // Allow some time for the message to be received
  ros::Duration(0.5).sleep();
  ros::spinOnce();

  EXPECT_TRUE(message_received);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}