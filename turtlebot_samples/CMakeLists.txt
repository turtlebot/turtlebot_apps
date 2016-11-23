cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_samples)

find_package(catkin REQUIRED COMPONENTS
  frontier_exploration
  turtlebot_navigation
)

# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot_samples
#  CATKIN_DEPENDS frontier_exploration frontier_exploration turtlebot_navigation
#  DEPENDS system_lib
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_rostest(test/frontier.test)
endif()