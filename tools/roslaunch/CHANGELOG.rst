^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roslaunch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.11 (2014-08-18)
--------------------
* fix the ROS_MASTER_URI environment variable logic on Windows (`#2 <https://github.com/windows/ros_comm/issues/2>`_)

1.10.10 (2014-06-16)
--------------------
* fix handling of if/unless attributes on args (`#437 <https://github.com/ros/ros_comm/issues/437>`_)

1.10.3 (2014-06-02)
-------------------

1.10.2 (2014-03-03)
-------------------

1.10.1 (2014-02-25)
-------------------
* add optional DEPENDENCIES argument to roslaunch_add_file_check()
* add explicit run dependency (`#347 <https://github.com/ros/ros_comm/issues/347>`_)

1.9.54 (2014-01-27)
-------------------
* add missing run/test dependencies on rosbuild to get ROS_ROOT environment variable

1.9.53 (2014-01-14)
-------------------

1.9.52 (2014-01-08)
-------------------

1.9.51 (2014-01-07)
-------------------
* fix roslaunch-check for unreleased wet dependencies (`#332 <https://github.com/ros/ros_comm/issues/332>`_)

1.9.50 (2013-10-04)
-------------------
* add option to disable terminal title setting
* fix roslaunch-check to handle more complex launch files

1.9.49 (2013-09-16)
-------------------

1.9.48 (2013-08-21)
-------------------
* update roslaunch to support ROS_NAMESPACE (`#58 <https://github.com/ros/ros_comm/issues/58>`_)
* make roslaunch relocatable (`ros/catkin#490 <https://github.com/ros/catkin/issues/490>`_)
* change roslaunch resolve order (`#256 <https://github.com/ros/ros_comm/issues/256>`_)
* fix roslaunch check script in install space (`#257 <https://github.com/ros/ros_comm/issues/257>`_)

1.9.47 (2013-07-03)
-------------------
* improve roslaunch completion to include launch file arguments (`#230 <https://github.com/ros/ros_comm/issues/230>`_)
* check for CATKIN_ENABLE_TESTING to enable configure without tests

1.9.46 (2013-06-18)
-------------------
* add CMake function roslaunch_add_file_check() (`#241 <https://github.com/ros/ros_comm/issues/241>`_)

1.9.45 (2013-06-06)
-------------------
* modified roslaunch $(find PKG) to consider path behind it for resolve strategy (`#233 <https://github.com/ros/ros_comm/pull/233>`_)
* add boolean attribute 'subst_value' to rosparam tag in launch files (`#218 <https://github.com/ros/ros_comm/issues/218>`_)
* add command line parameter to print out launch args
* fix missing import in arg_dump.py

1.9.44 (2013-03-21)
-------------------
* fix 'roslaunch --files' with non-unique anononymous ids (`#186 <https://github.com/ros/ros_comm/issues/186>`_)
* fix ROS_MASTER_URI for Windows

1.9.43 (2013-03-13)
-------------------
* implement process killer for Windows

1.9.42 (2013-03-08)
-------------------
* add option --skip-log-check (`#133 <https://github.com/ros/ros_comm/issues/133>`_)
* update API doc to list raised exceptions in config.py
* fix invocation of Python scripts under Windows (`#54 <https://github.com/ros/ros_comm/issues/54>`_)

1.9.41 (2013-01-24)
-------------------
* improve performance of $(find ...)

1.9.40 (2013-01-13)
-------------------
* fix 'roslaunch --pid=' when pointing to ROS_HOME but folder does not exist (`#43 <https://github.com/ros/ros_comm/issues/43>`_)
* fix 'roslaunch --pid=' to use shell expansion for the pid value (`#44 <https://github.com/ros/ros_comm/issues/44>`_)

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
