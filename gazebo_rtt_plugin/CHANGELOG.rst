^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_rtt_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2013-09-23)
------------------
* fixed segfault if marshalling service is not available
  for some reason
* only import packages using the ros.import() service
* minor fixes to remove some warnings in sdformat library
* use rtt_ros service to import ROS packages
* fixed linker error for static constants in RTT::ConnPolicy
* removed warnings due to deprecated sdf API calls

0.3.0 (2013-09-02)
------------------
* Catkinization of stack hector_gazebo
