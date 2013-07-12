file(GLOB pano_cv_srcs opencv.cpp mat.cpp numpy_to_mat.cpp)

add_library(pano_cv ${pano_cv_srcs})
target_link_libraries(pano_cv ${Boost_LIBRARIES})

# do this for python module importing
set_target_properties(pano_cv PROPERTIES PREFIX "")
