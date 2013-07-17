# this is the pano_py library/python module

file(GLOB pano_py_srcs *.cpp)

add_library(${PROJECT_NAME} ${pano_py_srcs})

target_link_libraries(${PROJECT_NAME} pano_cv ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

# Set python modules destination; it must be on PYTHONPATH

# WARN/TODO:
# catkin_destinations macro is required to set CATKIN_PACKAGE_PYTHON_DESTINATION, what is the right place to put
# python modules, but the current code expect to find libraries not inside its package but on global destination
catkin_destinations()

set_target_properties(${PROJECT_NAME}
  PROPERTIES PREFIX "")
set_target_properties(${PROJECT_NAME}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}
)