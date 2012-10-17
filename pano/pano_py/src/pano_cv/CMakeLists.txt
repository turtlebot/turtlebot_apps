file(GLOB pano_cv_srcs opencv.cpp mat.cpp numpy_to_mat.cpp)
rosbuild_add_library(pano_cv ${pano_cv_srcs})


#do this for python module importing
set_target_properties(pano_cv
  PROPERTIES
  PREFIX "")

rosbuild_link_boost(pano_cv python)
