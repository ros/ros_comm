^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package topic_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.13.4 (2017-11-02)
-------------------

1.13.3 (2017-10-25)
-------------------
* add initial_topic param (`#1199 <https://github.com/ros/ros_comm/issues/1199>`_)
* make demux more agile (`#1196 <https://github.com/ros/ros_comm/issues/1196>`_)
* add stealth mode for topic_tools/relay (`#1155 <https://github.com/ros/ros_comm/issues/1155>`_)

1.13.2 (2017-08-15)
-------------------

1.13.1 (2017-07-27)
-------------------

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
* add abstract class to implement connection based transport (`#713 <https://github.com/ros/ros_comm/pull/713>`_)

1.12.2 (2016-06-03)
-------------------

1.12.1 (2016-04-18)
-------------------
* use directory specific compiler flags (`#785 <https://github.com/ros/ros_comm/pull/785>`_)

1.12.0 (2016-03-18)
-------------------

1.11.18 (2016-03-17)
--------------------
* fix CMake warning about non-existing targets

1.11.17 (2016-03-11)
--------------------
* add --wait-for-start option to relay_field script (`#728 <https://github.com/ros/ros_comm/pull/728>`_)
* use boost::make_shared instead of new for constructing boost::shared_ptr (`#740 <https://github.com/ros/ros_comm/issues/740>`_)

1.11.16 (2015-11-09)
--------------------

1.11.15 (2015-10-13)
--------------------

1.11.14 (2015-09-19)
--------------------
* new tool "relay_field" which allows relay topic fields to another topic (`#639 <https://github.com/ros/ros_comm/pull/639>`_)
* allow transform to be used with ros arguments and in a launch file (`#644 <https://github.com/ros/ros_comm/issues/644>`_)
* add --wait-for-start option to transform script (`#646 <https://github.com/ros/ros_comm/pull/646>`_)

1.11.13 (2015-04-28)
--------------------

1.11.12 (2015-04-27)
--------------------

1.11.11 (2015-04-16)
--------------------

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
* Python 3 compatibility (`#426 <https://github.com/ros/ros_comm/issues/426>`_)

1.11.3 (2014-05-21)
-------------------
* add demux program and related scripts (`#407 <https://github.com/ros/ros_comm/issues/407>`_)

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* add transform tool allowing to perform Python operations between message fields taken from several topics (`ros/rosdistro#398 <https://github.com/ros/ros_comm/issues/398>`_)

1.11.0 (2014-03-04)
-------------------
* make rostest in CMakeLists optional (`ros/rosdistro#3010 <https://github.com/ros/rosdistro/issues/3010>`_)
* use catkin_install_python() to install Python scripts (`#361 <https://github.com/ros/ros_comm/issues/361>`_)

1.10.0 (2014-02-11)
-------------------
* remove use of __connection header

1.9.54 (2014-01-27)
-------------------

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

1.9.47 (2013-07-03)
-------------------
* check for CATKIN_ENABLE_TESTING to enable configure without tests

1.9.46 (2013-06-18)
-------------------

1.9.45 (2013-06-06)
-------------------

1.9.44 (2013-03-21)
-------------------
* fix install destination for dll's under Windows

1.9.43 (2013-03-13)
-------------------

1.9.42 (2013-03-08)
-------------------

1.9.41 (2013-01-24)
-------------------

1.9.40 (2013-01-13)
-------------------

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
