Test Secure ROS
---------------

This package provides some tests for Secure ROS.

Requirements 
^^^^^^^^^^^^
::
  sudo apt-get install python-pytest

Test authorization configuration file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This test will parse the authorization file and check for errors. 

You may provide the authorization file as an argument or using the ``ROS_AUTH_FILE`` environment variable. ::

  rosrun test_secure_ros test_config.py -a ros_auth.yaml

Test authorization configuration file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This test checks authorization functions based on an example configuration file ``conf/test.yaml``. ::

  python -m pytest test_parameters.py


Test Self
^^^^^^^^^

This test can be used to check if the secure ROS master correctly follows the rules provided by the authorization file. Make sure that all the nodes are running before running this test.

You may then run the test as follows. ::

  rosrun test_secure_ros test_self.py -a /path/to/simple_pubsub_ros_auth.yaml -I 

The test will load the authorization configuration YAML file and also obtain the ROS master URI (``ROS_MASTER_URI``) and the current IP address (``ROS_IP``) from environment variables. You may provide these values if the environment variables are not set or if you wish to override them using the appropriate options. The node then calls the XMLRPC methods on the ROS master to check if the responses from the master are consistent with the rules in the authorization file.

