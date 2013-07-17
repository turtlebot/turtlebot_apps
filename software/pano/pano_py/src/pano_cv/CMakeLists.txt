file(GLOB pano_cv_srcs opencv.cpp mat.cpp numpy_to_mat.cpp)

add_library(pano_cv ${pano_cv_srcs})
target_link_libraries(pano_cv ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

# Set python modules destination; it must be on PYTHONPATH

# WARN/TODO:
# catkin_destinations macro is required to set CATKIN_PACKAGE_PYTHON_DESTINATION, what is the right place to put
# python modules, but the current code expect to find libraries not inside its package but on global destination
#catkin_destinations()

set_target_properties(pano_cv
  PROPERTIES PREFIX "")
set_target_properties(pano_cv
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS pano_cv
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}
)
