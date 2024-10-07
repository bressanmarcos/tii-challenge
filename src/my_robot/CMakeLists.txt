cmake_minimum_required(VERSION 3.0.2)
project(my_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rostest
)

catkin_package()

# After catkin_package()
if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

catkin_add_gtest(test_talker tests/test_talker.cpp)
target_link_libraries(test_talker ${catkin_LIBRARIES})

catkin_add_gtest(test_listener tests/test_listener.cpp)
target_link_libraries(test_listener ${catkin_LIBRARIES})
