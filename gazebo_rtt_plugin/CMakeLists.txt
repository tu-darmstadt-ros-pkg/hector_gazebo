cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_find_ros_package(rtt)
find_package(OROCOS-RTT COMPONENTS rtt-marshalling HINTS "${rtt_PACKAGE_PATH}/../install")
add_definitions(-DRTT_STATIC)

if(OROCOS-RTT_FOUND)
  include(${OROCOS-RTT_USE_FILE})
  add_subdirectory(src)
else()
  message(WARNING "Orocos RTT was not found. Package ${PROJECT_NAME} will not be built.")
endif()
