^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosmaster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.14.4 (2020-02-20)
-------------------
* bump CMake minimum version to avoid CMP0048 warning (`#1869 <https://github.com/ros/ros_comm/issues/1869>`_)
* use thread local storage for caching instances of ServerProxy (`#1732 <https://github.com/ros/ros_comm/issues/1732>`_)
* use condition attributes to specify Python 2 and 3 dependencies (`#1792 <https://github.com/ros/ros_comm/issues/1792>`_)
* fix issue occuring during alternating calls of getParamCached and setParam (`#1439 <https://github.com/ros/ros_comm/issues/1439>`_)
* fix docstring in unregisterSubscriber (`#1553 <https://github.com/ros/ros_comm/issues/1553>`_)
* set correctly typed @apivalidate default return values (`#1472 <https://github.com/ros/ros_comm/issues/1472>`_)

1.14.3 (2018-08-06)
-------------------

1.14.2 (2018-06-06)
-------------------

1.14.1 (2018-05-21)
-------------------

1.14.0 (2018-05-21)
-------------------

1.13.6 (2018-02-05)
-------------------
* add TCP_INFO availability check (`#1211 <https://github.com/ros/ros_comm/issues/1211>`_)
* replace Thread.setDaemon() using new API (`#1276 <https://github.com/ros/ros_comm/issues/1276>`_)

1.13.5 (2017-11-09)
-------------------

1.13.4 (2017-11-02)
-------------------
* catch exception with `socket.TCP_INFO` on WSL (`#1212 <https://github.com/ros/ros_comm/issues/1212>`_, regression from 1.13.1)

1.13.3 (2017-10-25)
-------------------
* add --set-master-logger-level option for 'rosmaster' to output LOG_API (`#1180 <https://github.com/ros/ros_comm/issues/1180>`_)

1.13.2 (2017-08-15)
-------------------

1.13.1 (2017-07-27)
-------------------
* close CLOSE_WAIT sockets by default (`#1104 <https://github.com/ros/ros_comm/issues/1104>`_)

1.13.0 (2017-02-22)
-------------------

1.12.7 (2017-02-17)
-------------------
* add more logging to publisher update calls (`#979 <https://github.com/ros/ros_comm/issues/979>`_)

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
* use defusedxml to prevent common xml issues (`#782 <https://github.com/ros/ros_comm/pull/782>`_)

1.12.0 (2016-03-18)
-------------------

1.11.18 (2016-03-17)
--------------------

1.11.17 (2016-03-11)
--------------------

1.11.16 (2015-11-09)
--------------------
* add `-w` and `-t` options (`#687 <https://github.com/ros/ros_comm/pull/687>`_)

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

1.11.10 (2014-12-22)
--------------------
* fix closing sockets properly on node shutdown (`#495 <https://github.com/ros/ros_comm/issues/495>`_)

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
