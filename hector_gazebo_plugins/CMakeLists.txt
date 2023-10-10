cmake_minimum_required(VERSION 3.5)
project(hector_gazebo_plugins VERSION 0.5.3)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

## Find gazebo
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
# find_package(Boost REQUIRED COMPONENTS thread)
# include_directories(${Boost_INCLUDE_DIRS})

## Set includes
set(INCLUDE_DIRS include 
    ${sensor_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/srv
)
include_directories(${INCLUDE_DIRS})

set(LIBS
    gazebo_ros
    geographic_msgs 
    geometry_msgs
    sensor_msgs
    rclcpp
    std_msgs
    std_srvs
    tf2
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SetBias.srv
  srv/SetReferenceGeoPose.srv
  DEPENDENCIES geographic_msgs geometry_msgs
)


###########
## Build ##
###########

# Connect targets with locally defined messages
get_available_rmw_implementations(rmw_implementations2)
foreach(rmw_implementation ${rmw_implementations2})
  find_package("${rmw_implementation}" REQUIRED)
  get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
  foreach(typesupport_impl ${typesupport_impls})
    rosidl_get_typesupport_target(hector_gazebo_ros_imu_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
    rosidl_get_typesupport_target(hector_gazebo_ros_gps_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  endforeach()
endforeach()

set(HECTOR_GAZEBO_ROS_PLUGINS_LIST "")

# add_library(diffdrive_plugin_6w SHARED src/diffdrive_plugin_6w.cpp)
# target_link_libraries(diffdrive_plugin_6w ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

# add_library(diffdrive_plugin_multi_wheel SHARED src/diffdrive_plugin_multi_wheel.cpp)
# target_link_libraries(diffdrive_plugin_multi_wheel ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

# add_library(gazebo_ros_force_based_move SHARED src/gazebo_ros_force_based_move.cpp)
# target_link_libraries(gazebo_ros_force_based_move ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(hector_gazebo_reset_plugin SHARED src/reset_plugin.cpp)
target_link_libraries(hector_gazebo_reset_plugin ${GAZEBO_LIBRARIES})
list(APPEND HECTOR_GAZEBO_ROS_PLUGINS_LIST hector_gazebo_reset_plugin)
ament_target_dependencies(hector_gazebo_reset_plugin ${LIBS})

add_library(hector_gazebo_ros_imu SHARED src/gazebo_ros_imu.cpp)
target_link_libraries(hector_gazebo_ros_imu ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${hector_gazebo_ros_imu_typesupport_target})
list(APPEND HECTOR_GAZEBO_ROS_PLUGINS_LIST hector_gazebo_ros_imu)
ament_target_dependencies(hector_gazebo_ros_imu ${LIBS})

add_library(hector_gazebo_ros_magnetic SHARED src/gazebo_ros_magnetic.cpp)
target_link_libraries(hector_gazebo_ros_magnetic ${GAZEBO_LIBRARIES})
list(APPEND HECTOR_GAZEBO_ROS_PLUGINS_LIST hector_gazebo_ros_magnetic)
ament_target_dependencies(hector_gazebo_ros_magnetic ${LIBS})

add_library(hector_gazebo_ros_gps SHARED src/gazebo_ros_gps.cpp)
target_link_libraries(hector_gazebo_ros_gps ${GAZEBO_LIBRARIES} ${hector_gazebo_ros_gps_typesupport_target})
list(APPEND HECTOR_GAZEBO_ROS_PLUGINS_LIST hector_gazebo_ros_gps)
ament_target_dependencies(hector_gazebo_ros_gps ${LIBS})

add_library(hector_gazebo_ros_sonar SHARED src/gazebo_ros_sonar.cpp)
target_link_libraries(hector_gazebo_ros_sonar ${GAZEBO_LIBRARIES})
list(APPEND HECTOR_GAZEBO_ROS_PLUGINS_LIST hector_gazebo_ros_sonar)
ament_target_dependencies(hector_gazebo_ros_sonar ${LIBS})

# add_library(hector_servo_plugin SHARED src/servo_plugin.cpp)
# target_link_libraries(hector_servo_plugin ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})



# #############
# ## Install ##
# #############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${HECTOR_GAZEBO_ROS_PLUGINS_LIST}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# install(TARGETS
#   diffdrive_plugin_6w
#   diffdrive_plugin_multi_wheel
#   gazebo_ros_force_based_move
#   hector_gazebo_reset_plugin
#   hector_gazebo_ros_imu
#   hector_gazebo_ros_magnetic
#   hector_gazebo_ros_gps
#   hector_gazebo_ros_sonar
#   hector_servo_plugin
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

ament_package()
