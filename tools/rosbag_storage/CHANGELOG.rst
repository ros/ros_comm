^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag_storage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* fix compatibility with libconsole-bridge in Jessie, take 2

1.12.9 (2017-11-06)
-------------------
* fix compatibility with libconsole-bridge in Jessie (`#1219 <https://github.com/ros/ros_comm/issues/1219>`_, regression from 1.12.8)

1.12.8 (2017-11-06)
-------------------
* check if bzfile\_ and lz4s\_ handle is valid before reading/writing/closing (`#1183 <https://github.com/ros/ros_comm/issues/1183>`_)
* fix an out of bounds read in rosbag::View::iterator::increment() (`#1191 <https://github.com/ros/ros_comm/issues/1191>`_)
* replace usage deprecated console_bridge macros (`#1149 <https://github.com/ros/ros_comm/issues/1149>`_)
* fix whitespace warnings with g++ 7 (`#1138 <https://github.com/ros/ros_comm/issues/1138>`_)
* remove deprecated dynamic exception specifications (`#1137 <https://github.com/ros/ros_comm/issues/1137>`_)
* fix buffer overflow vulnerability (`#1092 <https://github.com/ros/ros_comm/issues/1092>`_)
* fix rosbag::View::iterator copy assignment operator (`#1017 <https://github.com/ros/ros_comm/issues/1017>`_)
* fix open mode on Windows (`#1005 <https://github.com/ros/ros_comm/pull/1005>`_)
* add swap function instead of copy constructor / assignment operator for rosbag::Bag (`#1000 <https://github.com/ros/ros_comm/issues/1000>`_)

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
* make Bag constructor explicit (`#835 <https://github.com/ros/ros_comm/pull/835>`_)

1.12.2 (2016-06-03)
-------------------

1.12.1 (2016-04-18)
-------------------
* use directory specific compiler flags (`#785 <https://github.com/ros/ros_comm/pull/785>`_)

1.12.0 (2016-03-18)
-------------------

1.11.18 (2016-03-17)
--------------------
* fix compiler warnings

1.11.17 (2016-03-11)
--------------------
* use boost::make_shared instead of new for constructing boost::shared_ptr (`#740 <https://github.com/ros/ros_comm/issues/740>`_)

1.11.16 (2015-11-09)
--------------------

1.11.15 (2015-10-13)
--------------------

1.11.14 (2015-09-19)
--------------------

1.11.13 (2015-04-28)
--------------------

1.11.12 (2015-04-27)
--------------------

1.11.11 (2015-04-16)
--------------------
* support large bagfiles (>2GB) on 32-bit systems (`#464 <https://github.com/ros/ros_comm/issues/464>`_)

1.11.10 (2014-12-22)
--------------------
* fix various defects reported by coverity

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
* convert to use console bridge from upstream debian package (`ros/rosdistro#4633 <https://github.com/ros/rosdistro/issues/4633>`_)

1.11.4 (2014-06-16)
-------------------

1.11.3 (2014-05-21)
-------------------

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* add lz4 compression to rosbag (Python and C++) (`#356 <https://github.com/ros/ros_comm/issues/356>`_)
* move rosbag dox to rosbag_storage (`#389 <https://github.com/ros/ros_comm/issues/389>`_)

1.11.0 (2014-03-04)
-------------------

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
* move several client library independent parts from ros_comm into roscpp_core, split rosbag storage specific stuff from client library usage (`#299 <https://github.com/ros/ros_comm/issues/299>`_)
