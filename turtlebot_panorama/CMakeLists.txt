cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_panorama)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp actionlib std_msgs std_srvs sensor_msgs geometry_msgs message_generation image_transport pano_ros)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${Eigen_INCLUDE_DIR})

# services
add_service_files(
  FILES TakePano.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)

# common commands for building c++ executables and libraries
add_executable(panorama src/panorama.cpp)
add_dependencies(panorama pano_ros_gencpp turtlebot_panorama_gencpp)
target_link_libraries(panorama ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES})


# What other packages will need to use this package
catkin_package(
    DEPENDS Boost Eigen
    CATKIN_DEPENDS roscpp actionlib std_msgs std_srvs sensor_msgs geometry_msgs message_runtime image_transport pano_ros
    INCLUDE_DIRS include
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS panorama
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executable scripts (Python etc.) for installation
install(PROGRAMS scripts/loop.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark anything (useful) else for installation
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
