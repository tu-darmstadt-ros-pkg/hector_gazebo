cmake_minimum_required(VERSION 3.5)
project(hector_gazebo_worlds VERSION 0.5.3)

## Find ament package
find_package(ament_cmake REQUIRED)


#############
## Install ##
#############

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  maps
  Media
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
