#include <gtest/gtest.h>
#include "std_msgs/String.h"

TEST(TestTalker, testMessageContent) {
  std_msgs::String msg;
  msg.data = "Hello World";
  EXPECT_EQ(msg.data, "Hello World");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
