cmake_minimum_required(VERSION 2.8.3)
project(pano_core)

find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   LIBRARIES pano_core
   INCLUDE_DIRS include
   CATKIN_DEPENDS cv_bridge roscpp
   DEPENDS Boost OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB pano_srcs src/*.cpp)
add_library(pano_core ${pano_srcs})

target_link_libraries(pano_core ${catkin_LIBRARIES})
target_link_libraries(pano_core ${Boost_LIBRARIES})
target_link_libraries(pano_core ${OpenCV_LIBRARIES})

# standalone apps
add_executable(stitcher apps/stitcher.cpp)
target_link_libraries(stitcher pano_core ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(video_stitcher apps/video_stitcher.cpp)
target_link_libraries(video_stitcher pano_core ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS pano_core stitcher
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS pano_core video_stitcher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(pano_core_utest test/utest.cpp)
target_link_libraries(pano_core_utest pano_core)
