^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.5 (2015-02-23)
------------------
* fill position_covariance in sensor_msgs/NavSatFix message from position error model in gazebo_ros_gps (fix #17)
* added dynamic_reconfigure servers to gps, magneto and sonar plugins
  The GPS plugin allows to configure the status and server flags in the output message,
  additionally to the error characteristics. This allows to simulate GPS dropouts.
* added dynamic_reconfigure server for IMU sensor model parameters
* fixed invocation of sensor model in gazebo_ros_imu and gazebo_ros_sonar to also respect the scale error
* calculate angular rate from quaternion difference directly
  This seems to be numerically more stable and removes the jitter in the angular rate signal.
* added initial bias
* added bias publisher to gazebo_ros_imu
  ...to compare hector_pose_estimation estimates with ground truth.
  Also renamed heading to yaw in gazebo_ros_imu and updated pseudo AHRS orientation calculation.
* added scale error to the sensor model and removed linearization in drift update
  The scale error is assumed to be constant and its value is loaded from the `scaleError` plugin parameter.
  The default scale error is 1.0 (no scale error).
  The value returned by the model is `(real_value * scale_error) + offset + drift + noise`.
* fixed wrong calculation of reference earth magnetic field vector if declination!=0
* Contributors: Johannes Meyer

0.3.4 (2014-09-01)
------------------
* replaced old copied license header in servo_plugin.cpp
* simplified attitude error calculation in gazebo_ros_imu (fixes #12)
* fixed calculation of vector-valued sensor errors and sensor error model resetting with non-zero initial drift
* linking against catkin_libraries
* Contributors: Johannes Meyer, Markus Achtelik

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
