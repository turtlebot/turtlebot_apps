cmake_minimum_required(VERSION 2.8.3)
project(pano_ros)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs actionlib_msgs message_generation)

catkin_python_setup()

# actions
add_action_files(
  FILES PanoCapture.action
        Stitch.action
)

# messages
add_message_files(
  FILES Pano.msg
)

# Generate added messages and actions with any dependencies listed here
generate_messages(
    DEPENDENCIES
        actionlib_msgs
        std_msgs
        sensor_msgs
)

# What other packages will need to use this package
catkin_package(
    CATKIN_DEPENDS std_msgs sensor_msgs actionlib_msgs message_runtime
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS nodes/capture_client.py
		 nodes/capture_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
