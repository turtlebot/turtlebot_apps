cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_actions)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp actionlib actionlib_msgs geometry_msgs message_generation tf cv_bridge image_transport image_geometry)
find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# actions
add_action_files(
    DIRECTORY action
    FILES FindFiducial.action
          TurtlebotMove.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
        actionlib_msgs
        std_msgs
        geometry_msgs
)

# building c++ executables and libraries
add_executable(turtlebot_move_action_server src/turtlebot_move_action_server.cpp)
target_link_libraries(turtlebot_move_action_server ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(turtlebot_move_action_server turtlebot_actions_gencpp)

add_executable(find_fiducial_pose src/find_fiducial_pose.cpp src/detect_calibration_pattern.cpp)
target_link_libraries(find_fiducial_pose ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(find_fiducial_pose turtlebot_actions_gencpp)


# what other packages will need to use this package
catkin_package(
    DEPENDS eigen opencv
    CATKIN_DEPENDS roscpp actionlib actionlib_msgs geometry_msgs message_runtime tf cv_bridge image_transport image_geometry turtlebot_bringup
    INCLUDE_DIRS include
)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS turtlebot_move_action_server find_fiducial_pose 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS scripts/test_fiducial.py
                 scripts/test_move.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
