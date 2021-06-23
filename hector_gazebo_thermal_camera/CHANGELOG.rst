^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_gazebo_thermal_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.4 (2021-06-23)
------------------

0.5.3 (2021-02-03)
------------------
* Updated CMake version and fixed CMP0048 for non-metapackages.
* Updated package format and maintainer information.
* Contributors: Stefan Fabian

0.5.2 (2020-12-17)
------------------
* Merge pull request `#64 <https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/issues/64>`_ from ms-iot/upstream_windows_fix
  [Windows][melodic-devel] Follow catkin guide to update the install path
* fix install path. (`#1 <https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/issues/1>`_)
* Merge pull request `#53 <https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/issues/53>`_ from enwaytech/ml/cmp0054_fix
  add CMP0045 cmake policy before gazebo include in CMakeLists.txt
* add CMP0045 cmake policy before gazebo include in CMakeLists.txt
* Contributors: Johannes Meyer, Matthias Loebach, Sean Yen

0.5.1 (2018-06-29)
------------------
* Merge pull request `#44 <https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/issues/44>`_ from esteve/gazebo8
  Use Gazebo 8 APIs to avoid deprecation warnings.
* hector_gazebo_plugins/hector_gazebo_thermal_camera: removed catkin_package(DEPENDS gazebo) declaration which was a no-op anyway
  See https://github.com/ros-simulation/gazebo_ros_pkgs/pull/537.
* Contributors: Esteve Fernandez, Johannes Meyer

0.5.0 (2016-06-24)
------------------
* Updated gazebo dependencies to version 7 for kinetic release
* hector_gazebo_thermal_camera: get rid of unused dependencies nodelet, dynamic_reconfigure, driver_base and image_transport
* Contributors: Johannes Meyer

0.4.1 (2016-06-24)
------------------
* see 0.3.8

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
