cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_sounds)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED)

catkin_package()

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS chirp.bash DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES cow.wav DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
