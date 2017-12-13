"""
"""

import TestAuth 

def test_setters( ):
  auth = TestAuth.TestAuth( "conf/test.yaml" )
  auth.check_param_setter( "/talker", auth.machine1, True )
  auth.check_param_setter( "/talker", auth.machine2, False )
  auth.check_param_setter( "/talker", auth.machine3, False )
  auth.check_param_setter( "/listener", auth.machine1, False )
  auth.check_param_setter( "/listener", auth.machine2, True )
  auth.check_param_setter( "/listener", auth.machine3, False )
  auth.check_param_setter( "/abc/def", auth.machine1, True )
  auth.check_param_setter( "/abc/def", auth.machine2, False )
  auth.check_param_setter( "/abc/def", auth.machine3, False )

def test_getters( ):
  auth = TestAuth.TestAuth( "conf/test.yaml" )
  auth.check_param_getter( "/talker", auth.machine1, True )
  auth.check_param_getter( "/talker", auth.machine2, False )
  auth.check_param_getter( "/talker", auth.machine3, False )
  auth.check_param_getter( "/listener", auth.machine1, False )
  auth.check_param_getter( "/listener", auth.machine2, True )
  auth.check_param_getter( "/listener", auth.machine3, False )
  auth.check_param_getter( "/abc/def", auth.machine1, True )
  auth.check_param_getter( "/abc/def", auth.machine2, True )
  auth.check_param_getter( "/abc/def", auth.machine3, False )


def test_prefix_setters( ):
  auth = TestAuth.TestAuth( "conf/test.yaml" )
  auth.check_param_setter( "/talker/a", auth.machine1, True )
  auth.check_param_setter( "/talker/a", auth.machine2, False )
  auth.check_param_setter( "/talker/listener", auth.machine1, True )
  auth.check_param_setter( "/talker/listener", auth.machine2, False )

  auth.check_param_setter( "/abc", auth.machine1, True )
  auth.check_param_setter( "/abc", auth.machine2, True )

  auth.check_param_setter( "/abc/def/g", auth.machine1, True )
  auth.check_param_setter( "/abc/def/g", auth.machine2, False )


def test_prefix_getters( ):
  auth = TestAuth.TestAuth( "conf/test.yaml" )
  auth.check_param_getter( "/talker/a", auth.machine1, True )
  auth.check_param_getter( "/talker/a", auth.machine2, False )
  auth.check_param_getter( "/talker/listener", auth.machine1, True )
  auth.check_param_getter( "/talker/listener", auth.machine2, False )

  auth.check_param_getter( "/abc", auth.machine1, True )
  auth.check_param_getter( "/abc", auth.machine2, True )

  auth.check_param_getter( "/abc/def/g", auth.machine1, True )
  auth.check_param_getter( "/abc/def/g", auth.machine2, True )
