^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rospy
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.10.10 (2014-06-16)
--------------------

1.10.3 (2014-06-02)
-------------------
* improve asynchonous publishing performance (`#373 <https://github.com/ros/ros_comm/issues/373>`_)
* allow custom error handlers for services (`#375 <https://github.com/ros/ros_comm/issues/375>`_)

1.10.2 (2014-03-03)
-------------------
* fix exception handling for queued connections (`#369 <https://github.com/ros/ros_comm/issues/369>`_)

1.10.1 (2014-02-25)
-------------------

1.9.54 (2014-01-27)
-------------------

1.9.53 (2014-01-14)
-------------------

1.9.52 (2014-01-08)
-------------------

1.9.51 (2014-01-07)
-------------------
* implement optional queueing for rospy publications (`#169 <https://github.com/ros/ros_comm/issues/169>`_)
* overwrite __repr__ for rospy.Duration and Time (`ros/genpy#24 <https://github.com/ros/genpy/issues/24>`_)
* add missing dependency on roscpp

1.9.50 (2013-10-04)
-------------------
* add support for python coverage tool to work in callbacks

1.9.49 (2013-09-16)
-------------------

1.9.48 (2013-08-21)
-------------------
* make rospy nodes killable while waiting for master (`#262 <https://github.com/ros/ros_comm/issues/262>`_)

1.9.47 (2013-07-03)
-------------------

1.9.46 (2013-06-18)
-------------------

1.9.45 (2013-06-06)
-------------------
* add missing run_depend on python-yaml
* allow configuration of ports for XML RPCs and TCP ROS
* fix race condition where rospy subscribers do not connect to all publisher
* fix closing and deregistering connection when connect fails (`#128 <https://github.com/ros/ros_comm/issues/128>`_)
* fix log level of RosOutHandler (`#210 <https://github.com/ros/ros_comm/issues/210>`_)

1.9.44 (2013-03-21)
-------------------

1.9.43 (2013-03-13)
-------------------

1.9.42 (2013-03-08)
-------------------
* make dependencies on rospy optional by refactoring RosStreamHandler to rosgraph (`#179 <https://github.com/ros/ros_comm/issues/179>`_)

1.9.41 (2013-01-24)
-------------------

1.9.40 (2013-01-13)
-------------------
* add colorization for rospy log output (`#3691 <https://code.ros.org/trac/ros/ticket/3691>`_)
* fix socket polling under Windows (`#3959 <https://code.ros.org/trac/ros/ticket/3959>`_)

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
