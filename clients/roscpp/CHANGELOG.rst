^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roscpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* fix compile problem with gcc 4.4 (`#302 <https://github.com/ros/ros_comm/issues/302>`_)

1.9.50 (2013-10-04)
-------------------

1.9.49 (2013-09-16)
-------------------
* add rosparam getter/setter for std::vector and std::map (`#279 <https://github.com/ros/ros_comm/issues/279>`_)

1.9.48 (2013-08-21)
-------------------

1.9.47 (2013-07-03)
-------------------

1.9.46 (2013-06-18)
-------------------

1.9.45 (2013-06-06)
-------------------
* improve handling of UDP transport, when fragmented packets are lost or arive out-of-order the connection is not dropped anymore, onle a single message is lost (`#226 <https://github.com/ros/ros_comm/issues/226>`_)
* fix missing generation of constant definitions for services (`ros/gencpp#2 <https://github.com/ros/gencpp/issues/2>`_)
* fix restoring thread context when callback throws an exception (`#219 <https://github.com/ros/ros_comm/issues/219>`_)
* fix calling PollManager::shutdown() repeatedly (`#217 <https://github.com/ros/ros_comm/issues/217>`_)

1.9.44 (2013-03-21)
-------------------
* fix install destination for dll's under Windows

1.9.43 (2013-03-13)
-------------------

1.9.42 (2013-03-08)
-------------------
* improve speed of message generation in dry packages (`#183 <https://github.com/ros/ros_comm/issues/183>`_)
* fix roscpp service call deadlock (`#149 <https://github.com/ros/ros_comm/issues/149>`_)
* fix freezing service calls when returning false (`#168 <https://github.com/ros/ros_comm/issues/168>`_)
* fix error message publishing wrong message type (`#178 <https://github.com/ros/ros_comm/issues/178>`_)
* fix missing explicit dependency on pthread (`#135 <https://github.com/ros/ros_comm/issues/135>`_)
* fix compiler warning about wrong comparison of message md5 hashes (`#165 <https://github.com/ros/ros_comm/issues/165>`_)

1.9.41 (2013-01-24)
-------------------
* allow sending data exceeding 2GB in chunks (`#4049 <https://code.ros.org/trac/ros/ticket/4049>`_)
* update getParam() doc (`#1460 <https://code.ros.org/trac/ros/ticket/1460>`_)
* add param::get(float) (`#3754 <https://code.ros.org/trac/ros/ticket/3754>`_)
* update inactive assert when publishing message with md5sum *, update related tests (`#3714 <https://code.ros.org/trac/ros/ticket/3714>`_)
* fix ros master retry timeout (`#4024 <https://code.ros.org/trac/ros/ticket/4024>`_)
* fix inactive assert when publishing message with wrong type (`#3714 <https://code.ros.org/trac/ros/ticket/3714>`_)

1.9.40 (2013-01-13)
-------------------

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
