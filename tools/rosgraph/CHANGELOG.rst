^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosgraph
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.6 (2014-07-10)
-------------------

1.11.5 (2014-06-24)
-------------------

1.11.4 (2014-06-16)
-------------------
* Python 3 compatibility (`#426 <https://github.com/ros/ros_comm/issues/426>`_, `#427 <https://github.com/ros/ros_comm/issues/427>`_, `#429 <https://github.com/ros/ros_comm/issues/429>`_)

1.11.3 (2014-05-21)
-------------------

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* add architecture_independent flag in package.xml (`#391 <https://github.com/ros/ros_comm/issues/391>`_)

1.11.0 (2014-03-04)
-------------------
* use catkin_install_python() to install Python scripts (`#361 <https://github.com/ros/ros_comm/issues/361>`_)

1.10.0 (2014-02-11)
-------------------

1.9.54 (2014-01-27)
-------------------

1.9.53 (2014-01-14)
-------------------

1.9.52 (2014-01-08)
-------------------

1.9.51 (2014-01-07)
-------------------
* allow different 127. addresses than 127.0.0.1 (`#315 <https://github.com/ros/ros_comm/issues/315>`_)
* work around for nose 1.3.0 bug (`#323 <https://github.com/ros/ros_comm/issues/323>`_)

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
* add warnings for obviously wrong environment variables ROS_HOSTNAME and ROS_IP (`#134 <https://github.com/ros/ros_comm/issues/134>`_)
* fix exception on netifaces.ifaddresses() (`#211 <https://github.com/ros/ros_comm/issues/211>`_, `#213 <https://github.com/ros/ros_comm/issues/213>`_) (regression from 1.9.42)

1.9.44 (2013-03-21)
-------------------

1.9.43 (2013-03-13)
-------------------

1.9.42 (2013-03-08)
-------------------
* replace custom code with Python module netifaces (`#130 <https://github.com/ros/ros_comm/issues/130>`_)
* make dependencies on rospy optional by refactoring RosStreamHandler to rosgraph (`#179 <https://github.com/ros/ros_comm/issues/179>`_)

1.9.41 (2013-01-24)
-------------------

1.9.40 (2013-01-13)
-------------------
* add colorization for rospy log output (`#3691 <https://code.ros.org/trac/ros/ticket/3691>`_)

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
