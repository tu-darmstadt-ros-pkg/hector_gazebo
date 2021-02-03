cmake_minimum_required(VERSION 3.5)
project(hector_gazebo_thermal_camera VERSION 0.5.3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS gazebo_plugins)
include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find OGRE and OGRE-Paging (required for CameraPlugin.hh in Gazebo 5)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
endif()
include_directories(${OGRE_INCLUDE_DIRS} ${OGRE-Paging_INCLUDE_DIRS})
link_directories(${OGRE_LIBRARY_DIRS} ${OGRE-Paging_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

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
    CATKIN_DEPENDS gazebo_plugins
    INCLUDE_DIRS include
    LIBRARIES
)

###########
## Build ##
###########

add_library(gazebo_ros_thermal_camera src/gazebo_ros_thermal_camera_plugin.cpp)
target_link_libraries(gazebo_ros_thermal_camera ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} CameraPlugin)

add_library(gazebo_ros_thermal_depth_camera src/gazebo_ros_thermal_depth_camera_plugin.cpp)
target_link_libraries(gazebo_ros_thermal_depth_camera ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} DepthCameraPlugin)

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
  gazebo_ros_thermal_camera
  gazebo_ros_thermal_depth_camera
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
