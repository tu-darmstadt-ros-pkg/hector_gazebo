cmake_minimum_required(VERSION 2.8.3)
project(hector_gazebo_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs geometry_msgs nav_msgs tf dynamic_reconfigure message_generation)
include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetBias.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/SensorModel.cfg
  cfg/GNSS.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf message_runtime
    INCLUDE_DIRS include
    LIBRARIES
)

###########
## Build ##
###########

add_library(diffdrive_plugin_6w src/diffdrive_plugin_6w.cpp)
target_link_libraries(diffdrive_plugin_6w ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(diffdrive_plugin_multi_wheel src/diffdrive_plugin_multi_wheel.cpp)
target_link_libraries(diffdrive_plugin_multi_wheel ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_force_based_move src/gazebo_ros_force_based_move.cpp)
target_link_libraries(gazebo_ros_force_based_move ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(hector_gazebo_reset_plugin src/reset_plugin.cpp)
target_link_libraries(hector_gazebo_reset_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(hector_gazebo_ros_imu src/gazebo_ros_imu.cpp)
target_link_libraries(hector_gazebo_ros_imu ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(hector_gazebo_ros_imu ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

add_library(hector_gazebo_ros_magnetic src/gazebo_ros_magnetic.cpp)
target_link_libraries(hector_gazebo_ros_magnetic ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(hector_gazebo_ros_magnetic ${PROJECT_NAME}_gencfg)

add_library(hector_gazebo_ros_gps src/gazebo_ros_gps.cpp)
target_link_libraries(hector_gazebo_ros_gps ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(hector_gazebo_ros_gps ${PROJECT_NAME}_gencfg)

add_library(hector_gazebo_ros_sonar src/gazebo_ros_sonar.cpp)
target_link_libraries(hector_gazebo_ros_sonar ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(hector_gazebo_ros_sonar ${PROJECT_NAME}_gencfg)

add_library(hector_servo_plugin src/servo_plugin.cpp)
target_link_libraries(hector_servo_plugin ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  diffdrive_plugin_6w
  diffdrive_plugin_multi_wheel
  gazebo_ros_force_based_move
  hector_gazebo_reset_plugin
  hector_gazebo_ros_imu
  hector_gazebo_ros_magnetic
  hector_gazebo_ros_gps
  hector_gazebo_ros_sonar
  hector_servo_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
