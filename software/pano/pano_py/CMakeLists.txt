cmake_minimum_required(VERSION 2.8.3)
project(pano_py)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS pano_core rospy roscpp std_msgs image_transport cv_bridge)
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${PYTHON_INCLUDE_PATH} ${OpenCV_INCLUDE_DIRS})

add_subdirectory(src/pano_cv)
add_subdirectory(src/pano_py)


# What other packages will need to use this package
catkin_package(
    DEPENDS Boost PythonLibs OpenCV
    CATKIN_DEPENDS pano_core rospy roscpp std_msgs image_transport cv_bridge
    INCLUDE_DIRS include
    LIBRARIES pano_cv pano_py
)