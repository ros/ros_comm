^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roslaunch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* fix NameError in launch error handling (`#1965 <https://github.com/ros/ros_comm/issues/1965>`_)

1.15.6 (2020-05-21)
-------------------

1.15.5 (2020-05-15)
-------------------
* add --sigint-timeout and --sigterm-timeout parameters (`#1937 <https://github.com/ros/ros_comm/issues/1937>`_)
* roslaunch-check: search dir recursively (`#1914 <https://github.com/ros/ros_comm/issues/1914>`_)
* sort printed nodes by namespace alphabetically (`#1934 <https://github.com/ros/ros_comm/issues/1934>`_)
* remove pycrypto import (not used) (`#1922 <https://github.com/ros/ros_comm/issues/1922>`_)

1.15.4 (2020-03-19)
-------------------

1.15.3 (2020-02-28)
-------------------

1.15.2 (2020-02-25)
-------------------

1.15.1 (2020-02-24)
-------------------
* use setuptools instead of distutils (`#1870 <https://github.com/ros/ros_comm/issues/1870>`_)

1.15.0 (2020-02-21)
-------------------
* update test to pass with old and new yaml (`#1893 <https://github.com/ros/ros_comm/issues/1893>`_)

1.14.4 (2020-02-20)
-------------------
* allow empty machine arg in node tag (`#1885 <https://github.com/ros/ros_comm/issues/1885>`_)
* use double quotes for portable roslaunch-check command (`#1883 <https://github.com/ros/ros_comm/issues/1883>`_)
* bump CMake minimum version to avoid CMP0048 warning (`#1869 <https://github.com/ros/ros_comm/issues/1869>`_)
* wrap env prefix with double quotes (`#1810 <https://github.com/ros/ros_comm/issues/1810>`_)
* add ignore default args option to roslaunch-check (`#1788 <https://github.com/ros/ros_comm/issues/1788>`_)
* add platform check for --args code path (`#1809 <https://github.com/ros/ros_comm/issues/1809>`_)
* [Windows] workaround for Python 2 xmlrpc performance issues (`#1872 <https://github.com/ros/ros_comm/issues/1872>`_)
* allow empty ssh password for remote launching (`#1826 <https://github.com/ros/ros_comm/issues/1826>`_)
* [Windows] escape drive as well as path separator (`#1815 <https://github.com/ros/ros_comm/issues/1815>`_)
* more Python 3 compatibility (`#1795 <https://github.com/ros/ros_comm/issues/1795>`_)
* use condition attributes to specify Python 2 and 3 dependencies (`#1792 <https://github.com/ros/ros_comm/issues/1792>`_)
* [Windows] skip `cat` related test cases on Windows build (`#1724 <https://github.com/ros/ros_comm/issues/1724>`_)
* [Windows] use taskkill to kill process tree (`#1725 <https://github.com/ros/ros_comm/issues/1725>`_)
* pass missing args (`#1733 <https://github.com/ros/ros_comm/issues/1733>`_)
* fix for roslaunch-check on Python 3
* roslaunch added --required option (`#1681 <https://github.com/ros/ros_comm/issues/1681>`_)
* more Python 3 compatibility (`#1783 <https://github.com/ros/ros_comm/issues/1783>`_)
* more Python 3 compatibility (`#1782 <https://github.com/ros/ros_comm/issues/1782>`_)
* switch to yaml.safe_load(_all) to prevent YAMLLoadWarning (`#1688 <https://github.com/ros/ros_comm/issues/1688>`_)
* fix $(dirname) for roslaunch-check (`#1624 <https://github.com/ros/ros_comm/issues/1624>`_)
* change how commands are executed (`#1628 <https://github.com/ros/ros_comm/issues/1628>`_)
* add option to hide summary from roslaunch output (`#1655 <https://github.com/ros/ros_comm/issues/1655>`_)
* make roslaunch-check respect arg remappings with command line argument (`#1653 <https://github.com/ros/ros_comm/issues/1653>`_)
* add POSIX flag for shlex.split() (`#1619 <https://github.com/ros/ros_comm/issues/1619>`_)
* respawn if process died while checking should_respawn() (`#1590 <https://github.com/ros/ros_comm/issues/1590>`_)
* add python prefix for python scripts when there is no .py extension (`#1589 <https://github.com/ros/ros_comm/issues/1589>`_)
* xmlloader: use continue instead of pass for args_only (`#1540 <https://github.com/ros/ros_comm/issues/1540>`_)
* fix various test problems (`#1601 <https://github.com/ros/ros_comm/issues/1601>`_)
* normalize strings to utf-8 before setting as environment variable (`#1593 <https://github.com/ros/ros_comm/issues/1593>`_)
* fix typos: awhile -> a while (`#1534 <https://github.com/ros/ros_comm/issues/1534>`_)
* add more useful helper text here to indicate that this might be a permission error (`#1568 <https://github.com/ros/ros_comm/issues/1568>`_)
* exclude unused args check if pass_all_args is set (`#1520 <https://github.com/ros/ros_comm/issues/1520>`_)
* add an option in XmlLoader to only load arg tags (`#1521 <https://github.com/ros/ros_comm/issues/1521>`_)
* update wiki.ros.org URLs (`#1536 <https://github.com/ros/ros_comm/issues/1536>`_)
* improve exception handling when resource is not found (`#1476 <https://github.com/ros/ros_comm/issues/1476>`_)
* fix issues when built or run on Windows (`#1466 <https://github.com/ros/ros_comm/issues/1466>`_)

1.14.3 (2018-08-06)
-------------------

1.14.2 (2018-06-06)
-------------------

1.14.1 (2018-05-21)
-------------------

1.14.0 (2018-05-21)
-------------------
* fix "pass_all_args" for roslaunch-check, add nosetest (`#1368 <https://github.com/ros/ros_comm/issues/1368>`_)
* add --log option to roslaunch (`#1330 <https://github.com/ros/ros_comm/issues/1330>`_)
* add substitution when loading yaml files (`#1354 <https://github.com/ros/ros_comm/issues/1354>`_)

1.13.6 (2018-02-05)
-------------------
* add process listeners to XML RPC server (`#1319 <https://github.com/ros/ros_comm/issues/1319>`_)
* pass through command-line args to the xmlloader when using the API (`#1115 <https://github.com/ros/ros_comm/issues/1115>`_)
* make master process explicitly 'required' for parent launch (`#1228 <https://github.com/ros/ros_comm/issues/1228>`_)
* remove unreachable exceptions (`#1260 <https://github.com/ros/ros_comm/issues/1260>`_)
* replace Thread.setDaemon() using new API (`#1276 <https://github.com/ros/ros_comm/issues/1276>`_)
* use roslaunch.core.printerrlog for printing error message (`#1193 <https://github.com/ros/ros_comm/issues/1193>`_, `#1317 <https://github.com/ros/ros_comm/issues/1317>`_)

1.13.5 (2017-11-09)
-------------------

1.13.4 (2017-11-02)
-------------------

1.13.3 (2017-10-25)
-------------------
* add --set-master-logger-level option for 'rosmaster' to output LOG_API (`#1180 <https://github.com/ros/ros_comm/issues/1180>`_)
* use defined error codes rather than hardcoded integers (`#1174 <https://github.com/ros/ros_comm/issues/1174>`_, `#1181 <https://github.com/ros/ros_comm/issues/1181>`_)
* fix parameter leaking into sibling scopes (`#1158 <https://github.com/ros/ros_comm/issues/1158>`_)
* avoid full stack trace for ResourceNotFound (`#1147 <https://github.com/ros/ros_comm/issues/1147>`_)
* remove mention of rosmake from error message (`#1140 <https://github.com/ros/ros_comm/issues/1140>`_)

1.13.2 (2017-08-15)
-------------------

1.13.1 (2017-07-27)
-------------------
* add $(dirname) to get directory of current launch file (`#1103 <https://github.com/ros/ros_comm/pull/1103>`_)
* clean the namespace to get rid of double or trailing forward slashes (`#1100 <https://github.com/ros/ros_comm/issues/1100>`_)
* only launch core nodes if master was launched by roslaunch (`#1098 <https://github.com/ros/ros_comm/pull/1098>`_)
* ensure pid file is removed on exit (`#1057 <https://github.com/ros/ros_comm/pull/1057>`_, `#1084 <https://github.com/ros/ros_comm/pull/1084>`_)
* add yaml type for param tag (`#1045 <https://github.com/ros/ros_comm/issues/1045>`_)
* ensure cwd exists (`#1031 <https://github.com/ros/ros_comm/pull/1031>`_)
* respect if/unless for roslaunch-check (`#998 <https://github.com/ros/ros_comm/pull/998>`_)

1.13.0 (2017-02-22)
-------------------

1.12.7 (2017-02-17)
-------------------
* improve error message for invalid tags (`#989 <https://github.com/ros/ros_comm/pull/989>`_)
* fix caching logic to improve performance (`#931 <https://github.com/ros/ros_comm/pull/931>`_)

1.12.6 (2016-10-26)
-------------------
* add USE_TEST_DEPENDENCIES option to roslaunch_add_file_check() (`#910 <https://github.com/ros/ros_comm/pull/910>`_)

1.12.5 (2016-09-30)
-------------------

1.12.4 (2016-09-19)
-------------------

1.12.3 (2016-09-17)
-------------------
* better naming for roslaunch check test results (`#897 <https://github.com/ros/ros_comm/pull/897>`_)
* support use_test_depends option for roslaunch-check (`#887 <https://github.com/ros/ros_comm/pull/887>`_)
* allow empty include (`#882 <https://github.com/ros/ros_comm/pull/882>`_)
* fix param command for Python 3 (`#840 <https://github.com/ros/ros_comm/pull/840>`_)

1.12.2 (2016-06-03)
-------------------
* support registering the same test multiple times with different arguments (`#814 <https://github.com/ros/ros_comm/pull/814>`_)
* fix passing multiple args to roslaunch_add_file_check (`#814 <https://github.com/ros/ros_comm/pull/814>`_)

1.12.1 (2016-04-18)
-------------------
* add support for Python expressions (`#784 <https://github.com/ros/ros_comm/pull/784>`_, `#793 <https://github.com/ros/ros_comm/pull/793>`_)

1.12.0 (2016-03-18)
-------------------

1.11.18 (2016-03-17)
--------------------

1.11.17 (2016-03-11)
--------------------
* improve roslaunch-check to not fail if recursive dependencies lack dependencies (`#730 <https://github.com/ros/ros_comm/pull/730>`_)
* add "pass_all_args" attribute to roslaunch "include" tag (`#710 <https://github.com/ros/ros_comm/pull/710>`_)
* fix a typo in unknown host error message (`#735 <https://github.com/ros/ros_comm/pull/735>`_)
* wait for param server to be available before trying to get param (`#711 <https://github.com/ros/ros_comm/pull/711>`_)

1.11.16 (2015-11-09)
--------------------
* add `-w` and `-t` options (`#687 <https://github.com/ros/ros_comm/pull/687>`_)
* fix missing minimum version for rospkg dependency (`#693 <https://github.com/ros/ros_comm/issues/693>`_)

1.11.15 (2015-10-13)
--------------------
* improve performance by reusing the rospack instance across nodes with the same default environment (`#682 <https://github.com/ros/ros_comm/pull/682>`_)

1.11.14 (2015-09-19)
--------------------
* add more information when test times out

1.11.13 (2015-04-28)
--------------------

1.11.12 (2015-04-27)
--------------------

1.11.11 (2015-04-16)
--------------------

1.11.10 (2014-12-22)
--------------------
* fix exception at roscore startup if python has IPv6 disabled (`#515 <https://github.com/ros/ros_comm/issues/515>`_)
* fix error handling (`#516 <https://github.com/ros/ros_comm/pull/516>`_)
* fix compatibility with paramiko 1.10.0 (`#498 <https://github.com/ros/ros_comm/pull/498>`_)

1.11.9 (2014-08-18)
-------------------
* fix usage of logger before it is initialized (`#490 <https://github.com/ros/ros_comm/issues/490>`_) (regression from 1.11.6)

1.11.8 (2014-08-04)
-------------------
* remove implicit rostest dependency and use rosunit instead (`#475 <https://github.com/ros/ros_comm/issues/475>`_)
* accept stdin input alongside files (`#472 <https://github.com/ros/ros_comm/issues/472>`_)

1.11.7 (2014-07-18)
-------------------
* fix the ROS_MASTER_URI environment variable logic on Windows (`#2 <https://github.com/windows/ros_comm/issues/2>`_)

1.11.6 (2014-07-10)
-------------------
* fix printing of non-ascii roslaunch parameters (`#454 <https://github.com/ros/ros_comm/issues/454>`_)
* add respawn_delay attribute to node tag in roslaunch (`#446 <https://github.com/ros/ros_comm/issues/446>`_)
* write traceback for exceptions in roslaunch to log file

1.11.5 (2014-06-24)
-------------------

1.11.4 (2014-06-16)
-------------------
* fix handling of if/unless attributes on args (`#437 <https://github.com/ros/ros_comm/issues/437>`_)
* improve parameter printing in roslaunch (`#89 <https://github.com/ros/ros_comm/issues/89>`_)
* Python 3 compatibility (`#426 <https://github.com/ros/ros_comm/issues/426>`_, `#427 <https://github.com/ros/ros_comm/issues/427>`_, `#429 <https://github.com/ros/ros_comm/issues/429>`_)

1.11.3 (2014-05-21)
-------------------

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* fix roslaunch anonymous function to generate the same output for the same input (`#297 <https://github.com/ros/ros_comm/issues/297>`_)
* add doc attribute to roslaunch arg tags (`#379 <https://github.com/ros/ros_comm/issues/379>`_)
* print parameter values in roslaunch (`#89 <https://github.com/ros/ros_comm/issues/89>`_)
* add architecture_independent flag in package.xml (`#391 <https://github.com/ros/ros_comm/issues/391>`_)

1.11.0 (2014-03-04)
-------------------
* use catkin_install_python() to install Python scripts (`#361 <https://github.com/ros/ros_comm/issues/361>`_)

1.10.0 (2014-02-11)
-------------------
* add optional DEPENDENCIES argument to roslaunch_add_file_check()
* add explicit run dependency (`#347 <https://github.com/ros/ros_comm/issues/347>`_)

1.9.54 (2014-01-27)
-------------------
* add missing run/test dependencies on rosbuild to get ROS_ROOT environment variable

1.9.53 (2014-01-14)
-------------------

1.9.52 (2014-01-08)
-------------------

1.9.51 (2014-01-07)
-------------------
* fix roslaunch-check for unreleased wet dependencies (`#332 <https://github.com/ros/ros_comm/issues/332>`_)

1.9.50 (2013-10-04)
-------------------
* add option to disable terminal title setting
* fix roslaunch-check to handle more complex launch files

1.9.49 (2013-09-16)
-------------------

1.9.48 (2013-08-21)
-------------------
* update roslaunch to support ROS_NAMESPACE (`#58 <https://github.com/ros/ros_comm/issues/58>`_)
* make roslaunch relocatable (`ros/catkin#490 <https://github.com/ros/catkin/issues/490>`_)
* change roslaunch resolve order (`#256 <https://github.com/ros/ros_comm/issues/256>`_)
* fix roslaunch check script in install space (`#257 <https://github.com/ros/ros_comm/issues/257>`_)

1.9.47 (2013-07-03)
-------------------
* improve roslaunch completion to include launch file arguments (`#230 <https://github.com/ros/ros_comm/issues/230>`_)
* check for CATKIN_ENABLE_TESTING to enable configure without tests

1.9.46 (2013-06-18)
-------------------
* add CMake function roslaunch_add_file_check() (`#241 <https://github.com/ros/ros_comm/issues/241>`_)

1.9.45 (2013-06-06)
-------------------
* modified roslaunch $(find PKG) to consider path behind it for resolve strategy (`#233 <https://github.com/ros/ros_comm/pull/233>`_)
* add boolean attribute 'subst_value' to rosparam tag in launch files (`#218 <https://github.com/ros/ros_comm/issues/218>`_)
* add command line parameter to print out launch args
* fix missing import in arg_dump.py

1.9.44 (2013-03-21)
-------------------
* fix 'roslaunch --files' with non-unique anononymous ids (`#186 <https://github.com/ros/ros_comm/issues/186>`_)
* fix ROS_MASTER_URI for Windows

1.9.43 (2013-03-13)
-------------------
* implement process killer for Windows

1.9.42 (2013-03-08)
-------------------
* add option --skip-log-check (`#133 <https://github.com/ros/ros_comm/issues/133>`_)
* update API doc to list raised exceptions in config.py
* fix invocation of Python scripts under Windows (`#54 <https://github.com/ros/ros_comm/issues/54>`_)

1.9.41 (2013-01-24)
-------------------
* improve performance of $(find ...)

1.9.40 (2013-01-13)
-------------------
* fix 'roslaunch --pid=' when pointing to ROS_HOME but folder does not exist (`#43 <https://github.com/ros/ros_comm/issues/43>`_)
* fix 'roslaunch --pid=' to use shell expansion for the pid value (`#44 <https://github.com/ros/ros_comm/issues/44>`_)

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
