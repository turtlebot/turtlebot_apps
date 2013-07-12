# this is the pano_py library/python module

file(GLOB pano_py_srcs *.cpp)

add_library(${PROJECT_NAME} ${pano_py_srcs})

target_link_libraries(${PROJECT_NAME} pano_cv ${Boost_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")