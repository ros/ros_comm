^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roswtf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.14.5 (2020-03-19)
-------------------

1.14.4 (2020-02-20)
-------------------
* add default ROS_MASTER_URI (`#1666 <https://github.com/ros/ros_comm/issues/1666>`_)
* bump CMake minimum version to avoid CMP0048 warning (`#1869 <https://github.com/ros/ros_comm/issues/1869>`_)
* do not try to run online checks if there are no roslaunch uris (`#1848 <https://github.com/ros/ros_comm/issues/1848>`_)
* more Python 3 compatibility (`#1796 <https://github.com/ros/ros_comm/issues/1796>`_)
* use condition attributes to specify Python 2 and 3 dependencies (`#1792 <https://github.com/ros/ros_comm/issues/1792>`_)
* print exception content to show better idea why loading plugin failed (`#1721 <https://github.com/ros/ros_comm/issues/1721>`_)
* duplicate test nodes which aren't available to other packages, add missing dependencies (`#1611 <https://github.com/ros/ros_comm/issues/1611>`_)
* query ipv6 only if specified (`#1596 <https://github.com/ros/ros_comm/issues/1596>`_)
* fix typos: awhile -> a while (`#1534 <https://github.com/ros/ros_comm/issues/1534>`_)
* improve msg replacement for 'No package or stack in context'. (`#1505 <https://github.com/ros/ros_comm/issues/1505>`_)

1.14.3 (2018-08-06)
-------------------

1.14.2 (2018-06-06)
-------------------

1.14.1 (2018-05-21)
-------------------

1.14.0 (2018-05-21)
-------------------
* warn if ROS_IP contains whitespace (`#1379 <https://github.com/ros/ros_comm/issues/1379>`_)

1.13.6 (2018-02-05)
-------------------

1.13.5 (2017-11-09)
-------------------

1.13.4 (2017-11-02)
-------------------

1.13.3 (2017-10-25)
-------------------

1.13.2 (2017-08-15)
-------------------

1.13.1 (2017-07-27)
-------------------
* improve roswtf tests (`#1101 <https://github.com/ros/ros_comm/pull/1101>`_, `#1102 <https://github.com/ros/ros_comm/pull/1102>`_)

1.13.0 (2017-02-22)
-------------------

1.12.7 (2017-02-17)
-------------------

1.12.6 (2016-10-26)
-------------------

1.12.5 (2016-09-30)
-------------------

1.12.4 (2016-09-19)
-------------------

1.12.3 (2016-09-17)
-------------------

1.12.2 (2016-06-03)
-------------------

1.12.1 (2016-04-18)
-------------------

1.12.0 (2016-03-18)
-------------------

1.11.18 (2016-03-17)
--------------------

1.11.17 (2016-03-11)
--------------------

1.11.16 (2015-11-09)
--------------------

1.11.15 (2015-10-13)
--------------------

1.11.14 (2015-09-19)
--------------------
* add optional dependency on geneus to make roswtf tests pass in jade

1.11.13 (2015-04-28)
--------------------

1.11.12 (2015-04-27)
--------------------

1.11.11 (2015-04-16)
--------------------
* support IPv6 addresses containing percentage symbols (`#585 <https://github.com/ros/ros_comm/issues/585>`_)

1.11.10 (2014-12-22)
--------------------

1.11.9 (2014-08-18)
-------------------

1.11.8 (2014-08-04)
-------------------

1.11.7 (2014-07-18)
-------------------

1.11.6 (2014-07-10)
-------------------

1.11.5 (2014-06-24)
-------------------

1.11.4 (2014-06-16)
-------------------
* Python 3 compatibility (`#426 <https://github.com/ros/ros_comm/issues/426>`_, `#427 <https://github.com/ros/ros_comm/issues/427>`_)

1.11.3 (2014-05-21)
-------------------

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* update roswtf test for upcoming rospack 2.2.3
* add architecture_independent flag in package.xml (`#391 <https://github.com/ros/ros_comm/issues/391>`_)

1.11.0 (2014-03-04)
-------------------
* make rostest in CMakeLists optional (`ros/rosdistro#3010 <https://github.com/ros/rosdistro/issues/3010>`_)

1.10.0 (2014-02-11)
-------------------

1.9.54 (2014-01-27)
-------------------
* fix roswtf checks to not require release-only python packages to be installed
* add missing run/test dependencies on rosbuild to get ROS_ROOT environment variable

1.9.53 (2014-01-14)
-------------------

1.9.52 (2014-01-08)
-------------------

1.9.51 (2014-01-07)
-------------------
* do not warn about not existing stacks folder in a catkin workspace

1.9.50 (2013-10-04)
-------------------

1.9.49 (2013-09-16)
-------------------

1.9.48 (2013-08-21)
-------------------

1.9.47 (2013-07-03)
-------------------
* check for CATKIN_ENABLE_TESTING to enable configure without tests

1.9.46 (2013-06-18)
-------------------

1.9.45 (2013-06-06)
-------------------

1.9.44 (2013-03-21)
-------------------
* fix ROS_ROOT check to access trailing 'rosbuild'

1.9.43 (2013-03-13)
-------------------

1.9.42 (2013-03-08)
-------------------

1.9.41 (2013-01-24)
-------------------

1.9.40 (2013-01-13)
-------------------
* add checks for pip packages and rosdep
* fix check for catkin_pkg
* fix for thread race condition causes incorrect graph connectivity analysis

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
