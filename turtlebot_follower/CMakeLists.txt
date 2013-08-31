cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_follower)

## Find catkin macros and libraries
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

catkin_package(
  INCLUDE_DIRS
  LIBRARIES turtlebot_follower
  CATKIN_DEPENDS nodelet pcl_ros roscpp visualization_msgs message_runtime dynamic_reconfigure
  DEPENDS Boost PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(turtlebot_follower src/follower.cpp)

add_dependencies(turtlebot_follower turtlebot_follower_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(turtlebot_follower
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS turtlebot_follower 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other required files for installation
install(DIRECTORY sim
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

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
