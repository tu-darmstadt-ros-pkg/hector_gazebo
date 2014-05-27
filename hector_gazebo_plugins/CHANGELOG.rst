^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.3 (2014-05-27)
------------------

0.3.2 (2014-03-30)
------------------
* diffdrive_plugin_multi_wheel: Fix wrong whell rotation calculation (Was only half speed of desired)
* diffdrive_plugin_multi_wheel: Optionally do not publish odometry via tf or as msg
* Fixed boost 1.53 issues
  Replaced boost::shared_dynamic_cast with boost::dynamic_pointer_cast
* Add servo plugin (used for vision box currently)
* Add catkin_LIBRARIES to linking for multiwheel plugin
* Some fixes to make diffdrive_plugin_multi_wheel work properly
* Work in progress of a diffdrive plugin supporting multiple wheels per side
* used updated API to get rid of warnings
* added topicName parameter back to gazebo_ros_magnetic
* hector_gazebo: deleted deprecated export sections from package.xml files
* abort with a fatal error if ROS is not yet initialized + minor code cleanup
* Contributors: Christopher Hrabia, Johannes Meyer, Richard Williams, Stefan Kohlbrecher, richardw347

0.3.1 (2013-09-23)
------------------
* fixed a bug in UpdateTimer class when updateRate and updatePeriod parameters are unset
* removed warnings due to deprecated sdf API calls

0.3.0 (2013-09-02)
------------------
* Catkinization of stack hector_gazebo
