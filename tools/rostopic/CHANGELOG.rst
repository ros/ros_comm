^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rostopic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.13.1 (2017-07-27)
-------------------
* fix rostopic prining long integers (`#1110 <https://github.com/ros/ros_comm/pull/1110>`_)

1.13.0 (2017-02-22)
-------------------

1.12.7 (2017-02-17)
-------------------

1.12.6 (2016-10-26)
-------------------
* fix typo of arg for _str_plot function (`#915 <https://github.com/ros/ros_comm/issues/915>`_)

1.12.5 (2016-09-30)
-------------------
* fix regression with rostopic echo for primitive fields from 1.12.3 (`#909 <https://github.com/ros/ros_comm/issues/909>`_)

1.12.4 (2016-09-19)
-------------------

1.12.3 (2016-09-17)
-------------------
* show topic field type with rostopic type (`#860 <https://github.com/ros/ros_comm/issues/860>`_)
* show stat for rostopic echo --noarr/nostr (`#724 <https://github.com/ros/ros_comm/pull/724>`_, `#872 <https://github.com/ros/ros_comm/pull/872>`_)
* add support for multiple topics in rostopic hz (`#712 <https://github.com/ros/ros_comm/pull/712>`_, `#886 <https://github.com/ros/ros_comm/pull/886>`_, `#888 <https://github.com/ros/ros_comm/pull/888>`_)
* more detailed help string for rostopic echo -p (`#816 <https://github.com/ros/ros_comm/issues/816>`_)

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
* add "rostopic delay" to measure message delay compared to the input from real world (`#719 <https://github.com/ros/ros_comm/pull/719>`_)
* add option to perform keyword substitution for messages published with "rostopic pub" (`#702 <https://github.com/ros/ros_comm/pull/702>`_)
* add wall-time option for rostopic hz (`#674 <https://github.com/ros/ros_comm/pull/674>`_)

1.11.16 (2015-11-09)
--------------------

1.11.15 (2015-10-13)
--------------------
* add warning to rostopic hz about simulated time (`#672 <https://github.com/ros/ros_comm/pull/672>`_)

1.11.14 (2015-09-19)
--------------------
* support specifying multiple array indices (`#606 <https://github.com/ros/ros_comm/pull/606>`_)
* fix string type check if variable is unicode

1.11.13 (2015-04-28)
--------------------

1.11.12 (2015-04-27)
--------------------
* fix command parsing for rosservice and rostopic to not accept arbitrary substrings of 'list' (`#609 <https://github.com/ros/ros_comm/issues/609>`_)

1.11.11 (2015-04-16)
--------------------

1.11.10 (2014-12-22)
--------------------
* add support for fixed-width floating-point and integer array values (`#400 <https://github.com/ros/ros_comm/issues/400>`_)

1.11.9 (2014-08-18)
-------------------

1.11.8 (2014-08-04)
-------------------
* fix handling of array index when being part of the topic (`#480 <https://github.com/ros/ros_comm/issues/480>`_)
* support splice arguments in 'rostopic echo' (`#480 <https://github.com/ros/ros_comm/issues/480>`_)

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
* add publisher queue_size to rostopic

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* add check for message fields when determining message type (`#376 <https://github.com/ros/ros_comm/issues/376>`_)

1.11.0 (2014-03-04)
-------------------
* make rostest in CMakeLists optional (`ros/rosdistro#3010 <https://github.com/ros/rosdistro/issues/3010>`_)
* use catkin_install_python() to install Python scripts (`#361 <https://github.com/ros/ros_comm/issues/361>`_)

1.10.0 (2014-02-11)
-------------------

1.9.54 (2014-01-27)
-------------------
* fix wrong time in csv export when using 'rostopic echo -p -b' (`#330 <https://github.com/ros/ros_comm/issues/330>`_)

1.9.53 (2014-01-14)
-------------------

1.9.52 (2014-01-08)
-------------------

1.9.51 (2014-01-07)
-------------------

1.9.50 (2013-10-04)
-------------------

1.9.49 (2013-09-16)
-------------------

1.9.48 (2013-08-21)
-------------------
* fix access to array fields by index (regression of `#242 <https://github.com/ros/ros_comm/issues/242>`_ in 1.9.47)

1.9.47 (2013-07-03)
-------------------
* fix 'rostopic echo' for submessages of type uint8[] (`#242 <https://github.com/ros/ros_comm/issues/242>`_)
* check for CATKIN_ENABLE_TESTING to enable configure without tests

1.9.46 (2013-06-18)
-------------------

1.9.45 (2013-06-06)
-------------------

1.9.44 (2013-03-21)
-------------------

1.9.43 (2013-03-13)
-------------------

1.9.42 (2013-03-08)
-------------------
* fix missing run_depend on rosbag (`#179 <https://github.com/ros/ros_comm/issues/179>`_)

1.9.41 (2013-01-24)
-------------------

1.9.40 (2013-01-13)
-------------------
* add support for boolean in 'rostopic -p' (`#3948 <https://code.ros.org/trac/ros/ticket/3948>`_)

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
