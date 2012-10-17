#this is the pano_py library/python module
file(GLOB pano_py_srcs *.cpp)
rosbuild_add_library(${PROJECT_NAME} ${pano_py_srcs})

target_link_libraries(${PROJECT_NAME} pano_cv)
rosbuild_link_boost(${PROJECT_NAME} python)

set_target_properties(${PROJECT_NAME}
  PROPERTIES
  PREFIX "")