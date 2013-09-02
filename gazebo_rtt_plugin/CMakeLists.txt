cmake_minimum_required(VERSION 2.8.3)
project(gazebo_rtt_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS hector_gazebo_plugins)
include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Orocos RTT
set(OROCOS_TARGET gnulinux)
find_package(OROCOS-RTT COMPONENTS rtt-marshalling)
add_definitions(-DRTT_STATIC)

if(NOT OROCOS-RTT_FOUND)
  message(WARNING "Orocos RTT was not found. Package ${PROJECT_NAME} will not be built.")
  return()
else()
  include(${OROCOS-RTT_USE_FILE})
endif()

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
    DEPENDS orocos-rtt-gnulinux
    CATKIN_DEPENDS
    INCLUDE_DIRS include
    LIBRARIES
)

###########
## Build ##
###########

add_library(gazebo_rtt_plugin src/gazebo_rtt_plugin.cpp src/gazebo_activity.cpp)
target_link_libraries(gazebo_rtt_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${OROCOS-RTT_LIBRARIES} ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY})

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
  gazebo_rtt_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
