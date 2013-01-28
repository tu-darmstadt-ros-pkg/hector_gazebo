rosbuild_add_library(gazebo_rtt_plugin gazebo_rtt_plugin.cpp gazebo_activity.cpp)
target_link_libraries(gazebo_rtt_plugin ${OROCOS-RTT_LIBRARIES} ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY})
