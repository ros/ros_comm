^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* use catkin_install_python() to install Python scripts (`#361 <https://github.com/ros/ros_comm/issues/361>`_)

1.10.0 (2014-02-11)
-------------------
* remove use of __connection header

1.9.54 (2014-01-27)
-------------------
* readd missing declaration of rosbag::createAdvertiseOptions (`#338 <https://github.com/ros/ros_comm/issues/338>`_)

1.9.53 (2014-01-14)
-------------------

1.9.52 (2014-01-08)
-------------------

1.9.51 (2014-01-07)
-------------------
* move several client library independent parts from ros_comm into roscpp_core, split rosbag storage specific stuff from client library usage (`#299 <https://github.com/ros/ros_comm/issues/299>`_)
* fix return value on platforms where char is unsigned.
* fix usage of boost include directories

1.9.50 (2013-10-04)
-------------------
* add chunksize option to rosbag record

1.9.49 (2013-09-16)
-------------------

1.9.48 (2013-08-21)
-------------------
* search for exported rosbag migration rules based on new package rosbag_migration_rule

1.9.47 (2013-07-03)
-------------------

1.9.46 (2013-06-18)
-------------------
* fix crash in bag migration (`#239 <https://github.com/ros/ros_comm/issues/239>`_)

1.9.45 (2013-06-06)
-------------------
* added option '--duration' to 'rosbag play' (`#121 <https://github.com/ros/ros_comm/issues/121>`_)
* fix missing newlines in rosbag error messages (`#237 <https://github.com/ros/ros_comm/issues/237>`_)
* fix flushing for tools like 'rosbag compress' (`#237 <https://github.com/ros/ros_comm/issues/237>`_)

1.9.44 (2013-03-21)
-------------------
* fix various issues on Windows (`#189 <https://github.com/ros/ros_comm/issues/189>`_)

1.9.43 (2013-03-13)
-------------------

1.9.42 (2013-03-08)
-------------------
* added option '--duration' to 'rosrun rosbag play' (`#121 <https://github.com/ros/ros_comm/issues/121>`_)
* add error message to rosbag when using same in and out file (`#171 <https://github.com/ros/ros_comm/issues/171>`_)

1.9.41 (2013-01-24)
-------------------

1.9.40 (2013-01-13)
-------------------
* fix bagsort script (`#42 <https://github.com/ros/ros_comm/issues/42>`_)

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
