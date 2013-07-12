cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_follower)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS nodelet pcl_ros roscpp visualization_msgs message_generation dynamic_reconfigure)
find_package(Boost REQUIRED)
find_package(PCL)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
  ChangeState.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages()

generate_dynamic_reconfigure_options(cfg/Follower.cfg)
include_directories(include cfg/cpp)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES turtlebot_follower
  CATKIN_DEPENDS nodelet pcl_ros roscpp visualization_msgs message_runtime
  DEPENDS system_lib PCL
)

###########
## Build ##
###########





## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(turtlebot_follower src/follower.cpp)

## Declare a cpp executable
# add_executable(turtlebot_follower_node src/turtlebot_follower_node.cpp)

add_dependencies(turtlebot_follower turtlebot_follower_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(turtlebot_follower
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
install(TARGETS turtlebot_follower 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY sim
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(FILES nodelet.xml   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_turtlebot_follower.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
