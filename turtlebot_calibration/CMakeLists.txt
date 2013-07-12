cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_calibration)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs nav_msgs geometry_msgs python_orocos_kdl turtlebot_bringup)

catkin_python_setup()

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
  CATKIN_DEPENDS rospy sensor_msgs nav_msgs geometry_msgs python_orocos_kdl turtlebot_bringup
)

# Mark executable scripts (Python etc.) for installation
install(PROGRAMS src/turtlebot_calibration/calibrate.py
                 src/turtlebot_calibration/scan_to_angle.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)