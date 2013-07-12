cmake_minimum_required(VERSION 2.8.3)
project(pano_ros)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs actionlib actionlib_msgs image_transport cv_bridge pano_py)

include_directories(include ${OpenCV_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

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
    CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs actionlib actionlib_msgs image_transport cv_bridge pano_py
)