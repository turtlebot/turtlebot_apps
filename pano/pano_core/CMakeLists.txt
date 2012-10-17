cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

set(ROS_BUILD_STATIC_LIBS true)
set(ROS_BUILD_SHARED_LIBS false)

#uncomment/comment to toggle crazy fast brief matching with the popcount instruction
set(ROS_COMPILE_FLAGS ${ROS_COMPILE_FLAGS} "-msse4.2 -Wall")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#this is the pano_core library
file(GLOB pano_srcs src/*.cpp)
rosbuild_add_library(${PROJECT_NAME} ${pano_srcs})

#unit tests
rosbuild_add_gtest(utest test/utest.cpp)
target_link_libraries(utest pano_core)

#standalone apps
rosbuild_add_executable(stitcher apps/stitcher.cpp)
target_link_libraries(stitcher ${PROJECT_NAME})
rosbuild_link_boost(stitcher program_options)

rosbuild_add_executable(video_stitcher apps/video_stitcher.cpp)
target_link_libraries(video_stitcher ${PROJECT_NAME})
rosbuild_link_boost(video_stitcher program_options)
