cmake_minimum_required(VERSION 2.8.3)
project(hector_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros
  std_msgs 
  geometry_msgs 
  nav_msgs
  tf 
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hector_gazebo_gps_plugin
  CATKIN_DEPENDS gazebo_ros
  )


include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO_SYSTEM REQUIRED gazebo_ros)
  # pkg_check_modules(GAZEBO_ODE REQUIRED gazebo_ode)
  # pkg_check_modules(GAZEBO_TRANSPORT REQUIRED gazebo_transport)
  pkg_check_modules(XML libxml-2.0)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

set(cxx_flags)
foreach (item ${GAZEBO_SYSTEM_CFLAGS})
  set(cxx_flags "${cxx_flags} ${item}")
endforeach ()
message(status "\n\n${cxx_flags}\n\n")

set(ld_flags)
foreach (item ${GAZEBO_SYSTEM_LDFLAGS})
  set(ld_flags "${ld_flags} ${item}")
endforeach ()
message(status "\n\n${ld_flags}\n\n")

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   SetBias.srv
 )

include_directories(include ${catkin_INCLUDE_DIRS} ${} ${GAZEBO_SYSTEM_INCLUDE_DIRS})
add_library(hector_gazebo_gps_plugin src/gazebo_ros_gps.cpp)
set_target_properties(hector_gazebo_gps_plugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(hector_gazebo_gps_plugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(hector_gazebo_gps_plugin ${GAZEBO_SYSTEM_LIBRARIES} ${catkin_LIBRARIES})

