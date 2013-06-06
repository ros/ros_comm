^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosgraph
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
