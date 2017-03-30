"""
"""

from rosmaster.authorization import ROSMasterAuth
import argparse
import os
import sys


class TestAuth( ):
  def __init__( self, auth_file ):
    self.auth = ROSMasterAuth( auth_file )
    for ( key, val ) in self.auth.aliases.items():
        setattr( self, key, val[0] )
        print( "%s: %s" % ( key, getattr( self, key ) ) )

  def check_param_setter( self, param, ip, value ):
    assert self.auth.check_param_setter( param, ip ) == value 

  def check_param_getter( self, param, ip, value ):
    assert self.auth.check_param_getter( param, ip ) == value 

