^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roscpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.14.3 (2018-08-06)
-------------------
* add hasStarted() to Timer API (`#1464 <https://github.com/ros/ros_comm/issues/1464>`_)
* fix compiler warnings about unused variables (`#1428 <https://github.com/ros/ros_comm/issues/1428>`_)

1.14.2 (2018-06-06)
-------------------

1.14.1 (2018-05-21)
-------------------

1.14.0 (2018-05-21)
-------------------
* force a rebuild of the pollset on flag changes (`#1393 <https://github.com/ros/ros_comm/issues/1393>`_)
* fix integer overflow for oneshot timers (`#1382 <https://github.com/ros/ros_comm/issues/1382>`_)
* convert the period standard deviation in StatisticsLogger to Duration at the end (`#1361 <https://github.com/ros/ros_comm/issues/1361>`_)
* add time when timer expired to timer events (`#1130 <https://github.com/ros/ros_comm/issues/1130>`_)
* replace DCL pattern with static variable (`#1365 <https://github.com/ros/ros_comm/issues/1365>`_)
* add parameter to stop clients from generating rosout topics list (`#1241 <https://github.com/ros/ros_comm/issues/1241>`_)

1.13.6 (2018-02-05)
-------------------
* avoid recreating poll set (`#1281 <https://github.com/ros/ros_comm/pull/1281>`_)
* switch to using epoll (`#1217 <https://github.com/ros/ros_comm/pull/1217>`_)
* monotonic clock for callback queue timeouts (`#1250 <https://github.com/ros/ros_comm/pull/1250>`_)
* fix IPv6 initialization order (`#1262 <https://github.com/ros/ros_comm/issues/1262>`_)
* changed error message for single threaded spinner  (`#1164 <https://github.com/ros/ros_comm/pull/1164>`_)

1.13.5 (2017-11-09)
-------------------

1.13.4 (2017-11-02)
-------------------

1.13.3 (2017-10-25)
-------------------
* avoid unused parameter warning in TransportTCP (`#1195 <https://github.com/ros/ros_comm/issues/1195>`_)
* check if socket options are available before using them (`#1172 <https://github.com/ros/ros_comm/issues/1172>`_)

1.13.2 (2017-08-15)
-------------------
* only use CLOCK_MONOTONIC if not on OS X (`#1142 <https://github.com/ros/ros_comm/issues/1142>`_)
* xmlrpc_manager: use SteadyTime for timeout (`#1134 <https://github.com/ros/ros_comm/issues/1134>`_)
* ignore headers with zero stamp in statistics (`#1127 <https://github.com/ros/ros_comm/issues/1127>`_)

1.13.1 (2017-07-27)
-------------------
* add SteadyTimer, used in TimerManager (`#1014 <https://github.com/ros/ros_comm/issues/1014>`_)
* include missing header for writev() (`#1105 <https://github.com/ros/ros_comm/pull/1105>`_)
* clean the namespace to get rid of double or trailing forward slashes (`#1100 <https://github.com/ros/ros_comm/issues/1100>`_)
* add missing mutex lock for publisher links (`#1090 <https://github.com/ros/ros_comm/pull/1090>`_)
* fix race condition that lead to miss first message (`#1058 <https://github.com/ros/ros_comm/issues/1058>`_)
* fix bug in transport_tcp on Windows (`#1050 <https://github.com/ros/ros_comm/issues/1050>`_)
* add subscriber to connection log messages (`#1023 <https://github.com/ros/ros_comm/issues/1023>`_)
* avoid deleting XmlRpcClient while being used in another thread (`#1013 <https://github.com/ros/ros_comm/issues/1013>`_)

1.13.0 (2017-02-22)
-------------------
* remove support for multiple spinners on the same queue which existed only for backward compatibily (`#988 <https://github.com/ros/ros_comm/pull/988>`_)

1.12.7 (2017-02-17)
-------------------
* move connection specific log message to new name roscpp_internal.connections (`#980 <https://github.com/ros/ros_comm/pull/980>`_)
* move headers to include/xmlrpcpp (`#962 <https://github.com/ros/ros_comm/issues/962>`_)
* fix UDP block number when EAGAIN or EWOULDBLOCK (`#957 <https://github.com/ros/ros_comm/issues/957>`_)
* fix return code of master execute function (`#938 <https://github.com/ros/ros_comm/pull/938>`_)
* change WallTimerEvent from class to struct (`#924 <https://github.com/ros/ros_comm/pull/924>`_)

1.12.6 (2016-10-26)
-------------------

1.12.5 (2016-09-30)
-------------------

1.12.4 (2016-09-19)
-------------------

1.12.3 (2016-09-17)
-------------------
* fix multi-threaded spinning (`#867 <https://github.com/ros/ros_comm/pull/867>`_)
* fix static destruction order (`#871 <https://github.com/ros/ros_comm/pull/871>`_)
* throw exception on ros::init with empty node name (`#894 <https://github.com/ros/ros_comm/pull/894>`_)
* improve debug message when queue is full (`#818 <https://github.com/ros/ros_comm/issues/818>`_)

1.12.2 (2016-06-03)
-------------------
* improve stacktrace for exceptions thrown in callbacks (`#811 <https://github.com/ros/ros_comm/pull/811>`_)
* fix segfault if creating outgoing UDP transport fails (`#807 <https://github.com/ros/ros_comm/pull/807>`_)

1.12.1 (2016-04-18)
-------------------
* use directory specific compiler flags (`#785 <https://github.com/ros/ros_comm/pull/785>`_)

1.12.0 (2016-03-18)
-------------------
* improve TopicManager::instance (`#770 <https://github.com/ros/ros_comm/issues/770>`_)
* change return value of param() to bool (`#753 <https://github.com/ros/ros_comm/issues/753>`_)

1.11.18 (2016-03-17)
--------------------
* fix CMake warning about non-existing targets

1.11.17 (2016-03-11)
--------------------
* fix order of argument in SubscriberLink interface to match actual implemenation (`#701 <https://github.com/ros/ros_comm/issues/701>`_)
* add method for getting all the parameters from the parameter server as implemented in the rospy client (`#739 <https://github.com/ros/ros_comm/issues/739>`_)
* use boost::make_shared instead of new for constructing boost::shared_ptr (`#740 <https://github.com/ros/ros_comm/issues/740>`_)
* fix max elements param for statistics window (`#750 <https://github.com/ros/ros_comm/issues/750>`_)
* improve NodeHandle constructor documentation (`#692 <https://github.com/ros/ros_comm/issues/692>`_)

1.11.16 (2015-11-09)
--------------------
* add getROSArg function (`#694 <https://github.com/ros/ros_comm/pull/694>`_)

1.11.15 (2015-10-13)
--------------------
* fix crash in onRetryTimer() callback (`#577 <https://github.com/ros/ros_comm/issues/577>`_)

1.11.14 (2015-09-19)
--------------------
* add optional reset argument to Timer::setPeriod() (`#590 <https://github.com/ros/ros_comm/issues/590>`_)
* add getParam() and getParamCached() for float (`#621 <https://github.com/ros/ros_comm/issues/621>`_, `#623 <https://github.com/ros/ros_comm/issues/623>`_)
* use explicit bool cast to compile with C++11 (`#632 <https://github.com/ros/ros_comm/pull/632>`_)

1.11.13 (2015-04-28)
--------------------

1.11.12 (2015-04-27)
--------------------

1.11.11 (2015-04-16)
--------------------
* fix memory leak in transport constructor (`#570 <https://github.com/ros/ros_comm/pull/570>`_)
* fix computation of stddev in statistics (`#556 <https://github.com/ros/ros_comm/pull/556>`_)
* fix empty connection header topic (`#543 <https://github.com/ros/ros_comm/issues/543>`_)
* alternative API to get parameter values (`#592 <https://github.com/ros/ros_comm/pull/592>`_)
* add getCached() for float parameters (`#584 <https://github.com/ros/ros_comm/pull/584>`_)

1.11.10 (2014-12-22)
--------------------
* fix various defects reported by coverity
* fix comment (`#529 <https://github.com/ros/ros_comm/issues/529>`_)
* improve Android support (`#518 <https://github.com/ros/ros_comm/pull/518>`_)

1.11.9 (2014-08-18)
-------------------
* add accessor to expose whether service is persistent (`#489 <https://github.com/ros/ros_comm/issues/489>`_)
* populate delivered_msgs field of TopicStatistics message (`#486 <https://github.com/ros/ros_comm/issues/486>`_)

1.11.8 (2014-08-04)
-------------------
* fix C++11 compatibility issue (`#483 <https://github.com/ros/ros_comm/issues/483>`_)

1.11.7 (2014-07-18)
-------------------
* fix segfault due to accessing a NULL pointer for some network interfaces (`#465 <https://github.com/ros/ros_comm/issues/465>`_) (regression from 1.11.6)

1.11.6 (2014-07-10)
-------------------
* check ROS_HOSTNAME for localhost / ROS_IP for 127./::1 and prevent connections from other hosts in that case (`#452 <https://github.com/ros/ros_comm/issues/452>`_)

1.11.5 (2014-06-24)
-------------------
* improve handling dropped connections (`#434 <https://github.com/ros/ros_comm/issues/434>`_)
* add header needed for Android (`#441 <https://github.com/ros/ros_comm/issues/441>`_)
* fix typo for parameter used for statistics (`#448 <https://github.com/ros/ros_comm/issues/448>`_)

1.11.4 (2014-06-16)
-------------------

1.11.3 (2014-05-21)
-------------------

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* update API to use boost::signals2 (`#267 <https://github.com/ros/ros_comm/issues/267>`_)
* only update param cache when being subscribed (`#351 <https://github.com/ros/ros_comm/issues/351>`_)
* ensure to remove delete parameters completely
* invalidate cached parent parameters when namespace parameter is set / changes (`#352 <https://github.com/ros/ros_comm/issues/352>`_)
* add optional topic/connection statistics (`#398 <https://github.com/ros/ros_comm/issues/398>`_)
* add transport information in SlaveAPI::getBusInfo() for roscpp & rospy (`#328 <https://github.com/ros/ros_comm/issues/328>`_)
* add AsyncSpinner::canStart() to check if a spinner can be started

1.11.0 (2014-03-04)
-------------------
* allow getting parameters with name '/' (`#313 <https://github.com/ros/ros_comm/issues/313>`_)
* support for /clock remapping (`#359 <https://github.com/ros/ros_comm/issues/359>`_)
* suppress boost::signals deprecation warning (`#362 <https://github.com/ros/ros_comm/issues/362>`_)
* use catkin_install_python() to install Python scripts (`#361 <https://github.com/ros/ros_comm/issues/361>`_)

1.10.0 (2014-02-11)
-------------------
* remove use of __connection header

1.9.54 (2014-01-27)
-------------------
* fix return value of pubUpdate() (`#334 <https://github.com/ros/ros_comm/issues/334>`_)
* fix handling optional third xml rpc response argument (`#335 <https://github.com/ros/ros_comm/issues/335>`_)

1.9.53 (2014-01-14)
-------------------

1.9.52 (2014-01-08)
-------------------

1.9.51 (2014-01-07)
-------------------
* move several client library independent parts from ros_comm into roscpp_core, split rosbag storage specific stuff from client library usage (`#299 <https://github.com/ros/ros_comm/issues/299>`_)
* add missing version dependency on roscpp_core stuff (`#299 <https://github.com/ros/ros_comm/issues/299>`_)
* remove log4cxx dependency from roscpp, using new agnostic interface from rosconsole
* fix compile problem with gcc 4.4 (`#302 <https://github.com/ros/ros_comm/issues/302>`_)
* fix clang warnings
* fix usage of boost include directories

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
* update inactive assert when publishing message with md5sum "*", update related tests (`#3714 <https://code.ros.org/trac/ros/ticket/3714>`_)
* fix ros master retry timeout (`#4024 <https://code.ros.org/trac/ros/ticket/4024>`_)
* fix inactive assert when publishing message with wrong type (`#3714 <https://code.ros.org/trac/ros/ticket/3714>`_)

1.9.40 (2013-01-13)
-------------------

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
