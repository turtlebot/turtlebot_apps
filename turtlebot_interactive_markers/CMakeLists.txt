cmake_minimum_required(VERSION 2.8.3)

project(turtlebot_interactive_markers)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        visualization_msgs
                                        interactive_markers)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(turtlebot_marker_server src/turtlebot_marker_server.cpp)

target_link_libraries(turtlebot_marker_server ${catkin_LIBRARIES})

install(TARGETS turtlebot_marker_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



