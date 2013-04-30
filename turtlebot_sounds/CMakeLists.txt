cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_sounds)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_package()

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  chirp.bash
  cow.wav
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

