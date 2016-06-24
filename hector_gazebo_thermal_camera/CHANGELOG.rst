^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_gazebo_thermal_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2016-06-24)
------------------
* Merge branch 'indigo-devel' into jade-devel
  Conflicts:
  hector_gazebo/CHANGELOG.rst
  hector_gazebo/package.xml
  hector_gazebo_plugins/CHANGELOG.rst
  hector_gazebo_plugins/package.xml
  hector_gazebo_thermal_camera/CHANGELOG.rst
  hector_gazebo_thermal_camera/package.xml
  hector_gazebo_worlds/CHANGELOG.rst
  hector_gazebo_worlds/package.xml
  hector_sensors_gazebo/CHANGELOG.rst
  hector_sensors_gazebo/package.xml
* Add Cmake flags for C++11
* 0.4.0
* Added proper dependencies for jade and gazebo5. Now compiles and works for gazebo5
* Contributors: Johannes Meyer, L0g1x, Romain Reignier

0.4.0 (2015-11-07)
------------------
* Added proper dependencies for jade and gazebo5. Now compiles and works for gazebo5
* Contributors: L0g1x

0.3.8 (2016-06-24)
------------------
* Compatible with gazebo7
* Add Cmake flags for C++11
* Contributors: Nate Koenig, Romain Reignier

0.3.7 (2015-11-07)
------------------
* hector_gazebo_plugins/hector_gazebo_thermal_camera: switch to cmake configuration for gazebo and added OGRE include directories required for CameraPlugin.hh
* Contributors: Johannes Meyer

0.3.6 (2015-03-21)
------------------

0.3.5 (2015-02-23)
------------------

0.3.4 (2014-09-01)
------------------

0.3.3 (2014-05-27)
------------------
* Fix bad bug with index access
* Contributors: Stefan Kohlbrecher

0.3.2 (2014-03-30)
------------------
* added missing dependency to roscpp
* hector_gazebo: deleted deprecated export sections from package.xml files
* Contributors: Johannes Meyer

0.3.1 (2013-09-23)
------------------
* fixed image_connect_count_ compile issues with the latest gazebo_plugins version 2.3.2

0.3.0 (2013-09-02)
------------------
* Catkinization of stack hector_gazebo
