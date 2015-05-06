cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_navigation)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS tf roscpp sensor_msgs)

# What other packages will need to use this package
catkin_package(
    CATKIN_DEPENDS tf roscpp sensor_msgs
)

catkin_add_env_hooks(25.turtlebot-navigation SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)


###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

# Add_executables
add_executable(laser_footprint_filter src/laser_footprint_filter.cpp)
target_link_libraries(laser_footprint_filter ${catkin_LIBRARIES})


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS laser_footprint_filter
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark anything (useful) else for installation
install(DIRECTORY laser
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY maps
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
