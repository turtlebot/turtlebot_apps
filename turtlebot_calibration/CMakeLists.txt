cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_calibration)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

# Messages
add_message_files(
  FILES ScanAngle.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)

# What other packages will need to use this package
catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

# Mark executable scripts (Python etc.) for installation
install(PROGRAMS src/turtlebot_calibration/calibrate.py
                 src/turtlebot_calibration/scan_to_angle.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
