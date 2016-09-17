^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roslaunch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.12.3 (2016-09-17)
-------------------
* better naming for roslaunch check test results (`#897 <https://github.com/ros/ros_comm/pull/897>`_)
* support use_test_depends option for roslaunch-check (`#887 <https://github.com/ros/ros_comm/pull/887>`_)
* allow empty include (`#882 <https://github.com/ros/ros_comm/pull/882>`_)
* fix param command for Python 3 (`#840 <https://github.com/ros/ros_comm/pull/840>`_)

1.12.2 (2016-06-03)
-------------------
* support registering the same test multiple times with different arguments (`#814 <https://github.com/ros/ros_comm/pull/814>`_)
* fix passing multiple args to roslaunch_add_file_check (`#814 <https://github.com/ros/ros_comm/pull/814>`_)

1.12.1 (2016-04-18)
-------------------
* add support for Python expressions (`#784 <https://github.com/ros/ros_comm/pull/784>`_, `#793 <https://github.com/ros/ros_comm/pull/793>`_)

1.12.0 (2016-03-18)
-------------------

1.11.18 (2016-03-17)
--------------------

1.11.17 (2016-03-11)
--------------------
* improve roslaunch-check to not fail if recursive dependencies lack dependencies (`#730 <https://github.com/ros/ros_comm/pull/730>`_)
* add "pass_all_args" attribute to roslaunch "include" tag (`#710 <https://github.com/ros/ros_comm/pull/710>`_)
* fix a typo in unknown host error message (`#735 <https://github.com/ros/ros_comm/pull/735>`_)
* wait for param server to be available before trying to get param (`#711 <https://github.com/ros/ros_comm/pull/711>`_)

1.11.16 (2015-11-09)
--------------------
* add `-w` and `-t` options (`#687 <https://github.com/ros/ros_comm/pull/687>`_)
* fix missing minimum version for rospkg dependency (`#693 <https://github.com/ros/ros_comm/issues/693>`_)

1.11.15 (2015-10-13)
--------------------
* improve performance by reusing the rospack instance across nodes with the same default environment (`#682 <https://github.com/ros/ros_comm/pull/682>`_)

1.11.14 (2015-09-19)
--------------------
* add more information when test times out

1.11.13 (2015-04-28)
--------------------

1.11.12 (2015-04-27)
--------------------

1.11.11 (2015-04-16)
--------------------

1.11.10 (2014-12-22)
--------------------
* fix exception at roscore startup if python has IPv6 disabled (`#515 <https://github.com/ros/ros_comm/issues/515>`_)
* fix error handling (`#516 <https://github.com/ros/ros_comm/pull/516>`_)
* fix compatibility with paramiko 1.10.0 (`#498 <https://github.com/ros/ros_comm/pull/498>`_)

1.11.9 (2014-08-18)
-------------------
* fix usage of logger before it is initialized (`#490 <https://github.com/ros/ros_comm/issues/490>`_) (regression from 1.11.6)

1.11.8 (2014-08-04)
-------------------
* remove implicit rostest dependency and use rosunit instead (`#475 <https://github.com/ros/ros_comm/issues/475>`_)
* accept stdin input alongside files (`#472 <https://github.com/ros/ros_comm/issues/472>`_)

1.11.7 (2014-07-18)
-------------------
* fix the ROS_MASTER_URI environment variable logic on Windows (`#2 <https://github.com/windows/ros_comm/issues/2>`_)

1.11.6 (2014-07-10)
-------------------
* fix printing of non-ascii roslaunch parameters (`#454 <https://github.com/ros/ros_comm/issues/454>`_)
* add respawn_delay attribute to node tag in roslaunch (`#446 <https://github.com/ros/ros_comm/issues/446>`_)
* write traceback for exceptions in roslaunch to log file

1.11.5 (2014-06-24)
-------------------

1.11.4 (2014-06-16)
-------------------
* fix handling of if/unless attributes on args (`#437 <https://github.com/ros/ros_comm/issues/437>`_)
* improve parameter printing in roslaunch (`#89 <https://github.com/ros/ros_comm/issues/89>`_)
* Python 3 compatibility (`#426 <https://github.com/ros/ros_comm/issues/426>`_, `#427 <https://github.com/ros/ros_comm/issues/427>`_, `#429 <https://github.com/ros/ros_comm/issues/429>`_)

1.11.3 (2014-05-21)
-------------------

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* fix roslaunch anonymous function to generate the same output for the same input (`#297 <https://github.com/ros/ros_comm/issues/297>`_)
* add doc attribute to roslaunch arg tags (`#379 <https://github.com/ros/ros_comm/issues/379>`_)
* print parameter values in roslaunch (`#89 <https://github.com/ros/ros_comm/issues/89>`_)
* add architecture_independent flag in package.xml (`#391 <https://github.com/ros/ros_comm/issues/391>`_)

1.11.0 (2014-03-04)
-------------------
* use catkin_install_python() to install Python scripts (`#361 <https://github.com/ros/ros_comm/issues/361>`_)

1.10.0 (2014-02-11)
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
