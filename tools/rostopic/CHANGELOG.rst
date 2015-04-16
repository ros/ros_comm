^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rostopic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
