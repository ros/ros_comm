^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.10 (2014-12-22)
--------------------
* add option to specify the minimum disk space at which recording is stopped (`#500 <https://github.com/ros/ros_comm/pull/500>`_)
* add convenience API to Python rosbag (`#508 <https://github.com/ros/ros_comm/issues/508>`_)
* fix delay on detecting a running rosmaster with use_sim_time set (`#532 <https://github.com/ros/ros_comm/pull/532>`_)

1.11.9 (2014-08-18)
-------------------

1.11.8 (2014-08-04)
-------------------

1.11.7 (2014-07-18)
-------------------

1.11.6 (2014-07-10)
-------------------
* fix rosbag record prefix (`#449 <https://github.com/ros/ros_comm/issues/449>`_)

1.11.5 (2014-06-24)
-------------------
* Fix typo in rosbag usage

1.11.4 (2014-06-16)
-------------------
* Python 3 compatibility (`#426 <https://github.com/ros/ros_comm/issues/426>`_, `#430 <https://github.com/ros/ros_comm/issues/430>`_)

1.11.3 (2014-05-21)
-------------------

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* add lz4 compression to rosbag (Python and C++) (`#356 <https://github.com/ros/ros_comm/issues/356>`_)
* fix rosbag record --node (`#357 <https://github.com/ros/ros_comm/issues/357>`_)
* move rosbag dox to rosbag_storage (`#389 <https://github.com/ros/ros_comm/issues/389>`_)

1.11.0 (2014-03-04)
-------------------
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
