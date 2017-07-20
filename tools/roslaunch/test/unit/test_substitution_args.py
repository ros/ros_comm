# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys

import rospkg

def test__arg():
    import random
    from roslaunch.substitution_args import _arg, ArgException, SubstitutionException
    
    ctx = { 'arg': {
            'foo': '12345',
            'bar': 'hello world',
            'baz': 'sim',
            'empty': '',
            }
            }
    
    # test invalid
    try:
        _arg('$(arg)', 'arg', [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test normal
    tests = [
        ('12345', ('$(arg foo)', 'arg foo', ['foo'], ctx)),
        ('', ('$(arg empty)', 'arg empty', ['empty'], ctx)),
        ('sim', ('$(arg baz)', 'arg baz', ['baz'], ctx)),
        
        # test with other args present, should only resolve match
        ('1234512345', ('$(arg foo)$(arg foo)', 'arg foo', ['foo'], ctx)),
        ('12345$(arg baz)', ('$(arg foo)$(arg baz)', 'arg foo', ['foo'], ctx)),            
        ('$(arg foo)sim', ('$(arg foo)$(arg baz)', 'arg baz', ['baz'], ctx)),            
        
        # test double-resolve safe
        ('12345', ('12345', 'arg foo', ['foo'], ctx)),            
        ]
        
    for result, test in tests:
        resolved, a, args, context = test
        assert result == _arg(resolved, a, args, context)

    #  - test that all fail if ctx is not set
    for result, test in tests:
        resolved, a, args, context = test
        try:
            _arg(resolved, a, args, {})
            assert False, "should have thrown"
        except ArgException as e:
            assert args[0] == str(e)
        try:
            _arg(resolved, a, args, {'arg': {}})
            assert False, "should have thrown"
        except ArgException as e:
            assert args[0] == str(e)

def test_resolve_args():
    from roslaunch.substitution_args import resolve_args, SubstitutionException

    r = rospkg.RosPack()
    roslaunch_dir = r.get_path('roslaunch')
    assert roslaunch_dir

    anon_context = {'foo': 'bar'}
    arg_context = {'fuga': 'hoge', 'car': 'cdr', 'arg': 'foo', 'True': 'False'}
    context = {'anon': anon_context, 'arg': arg_context, 'filename': '/path/to/file.launch'}
        
    tests = [
        ('$(find roslaunch)', roslaunch_dir),
        ('hello$(find roslaunch)', 'hello'+roslaunch_dir),
        ('$(find roslaunch )', roslaunch_dir),
        ('$$(find roslaunch )', '$'+roslaunch_dir),
        ('$( find roslaunch )', roslaunch_dir),
        ('$(find  roslaunch )', roslaunch_dir),
        ('$(find roslaunch)$(find roslaunch)', roslaunch_dir+os.sep+roslaunch_dir),
        ('$(find roslaunch)/foo/bar.xml', roslaunch_dir+os.sep+'foo'+os.sep+'bar.xml'),
        (r'$(find roslaunch)\foo\bar.xml $(find roslaunch)\bar.xml', roslaunch_dir+os.sep+'foo'+os.sep+'bar.xml '+roslaunch_dir+os.sep+'bar.xml'),
        ('$(find roslaunch)\\foo\\bar.xml more/stuff\\here', roslaunch_dir+os.sep+'foo'+os.sep+'bar.xml more/stuff\\here'),
        ('$(env ROS_ROOT)', os.environ['ROS_ROOT']),
        ('$(env ROS_ROOT)', os.environ['ROS_ROOT']),
        ('$(env ROS_ROOT )', os.environ['ROS_ROOT']),
        ('$(optenv ROS_ROOT)', os.environ['ROS_ROOT']),
        ('$(optenv ROS_ROOT)$(optenv ROS_ROOT)', os.environ['ROS_ROOT']+os.environ['ROS_ROOT']),
        ('$(optenv ROS_ROOT alternate text)', os.environ['ROS_ROOT']),
        ('$(optenv NOT_ROS_ROOT)', ''),
        ('$(optenv NOT_ROS_ROOT)more stuff', 'more stuff'),
        ('$(optenv NOT_ROS_ROOT alternate)', 'alternate'),
        ('$(optenv NOT_ROS_ROOT alternate text)', 'alternate text'),
        ('$(dirname)/foo', '/path/to/foo'),

        # #1776
        ('$(anon foo)', 'bar'),
        ('$(anon foo)/baz', 'bar/baz'),
        ('$(anon foo)/baz/$(anon foo)', 'bar/baz/bar'),

        # arg
        ('$(arg fuga)', 'hoge'),
        ('$(arg fuga)$(arg fuga)', 'hogehoge'),
        ('$(arg car)$(arg fuga)', 'cdrhoge'),
        ('$(arg fuga)hoge', 'hogehoge'),

        # $(eval ...) versions of those tests
        ("$(eval find('roslaunch'))", roslaunch_dir),
        ("$(eval env('ROS_ROOT'))", os.environ['ROS_ROOT']),
        ("$(eval optenv('ROS_ROOT', 'alternate text'))", os.environ['ROS_ROOT']),
        ("$(eval optenv('NOT_ROS_ROOT', 'alternate text'))", "alternate text"),
        ("$(eval optenv('NOT_ROS_ROOT'))", ""),
        ("$(eval anon('foo'))", 'bar'),
        ("$(eval arg('fuga'))", 'hoge'),
        ('$(eval arg("fuga"))', 'hoge'),
        ('$(eval arg("arg"))', 'foo'),
        ('$(eval arg("True"))', 'False'),
        ('$(eval 1==1)', 'True'),
        ('$(eval [0,1,2][1])', '1'),
        # test implicit arg access
        ('$(eval fuga)', 'hoge'),
        ('$(eval True)', 'True'),
        # math expressions
        ('$(eval round(sin(pi),1))', '0.0'),
        ('$(eval cos(0))', '1.0'),
        # str, map
        ("$(eval ''.join(map(str, [4,2])))", '42'),
    ]
    for arg, val in tests:
        assert val == resolve_args(arg, context=context), arg

    # more #1776
    r = resolve_args('$(anon foo)/bar')
    assert '/bar' in r
    assert not '$(anon foo)' in r
        
            
    # test against strings that should not match
    noop_tests = [
        '$(find roslaunch', '$find roslaunch', '', ' ', 'noop', 'find roslaunch', 'env ROS_ROOT', '$$', ')', '(', '()',
        None, 
        ]
    for t in noop_tests:
        assert t == resolve_args(t)
    failures = [
        '$((find roslaunch))',  '$(find $roslaunch)',
        '$(find)', '$(find roslaunch roslaunch)', '$(export roslaunch)',
        '$(env)', '$(env ROS_ROOT alternate)',
        '$(env NOT_SET)',
        '$(optenv)',
        '$(anon)',
        '$(anon foo bar)',            
        # Should fail without the supplied context.
        '$(dirname)'
        ]
    for f in failures:
        try:
            resolve_args(f)
            assert False, "resolve_args(%s) should have failed"%f
        except SubstitutionException: pass


def test_resolve_duplicate_anon():
    from roslaunch.config import ROSLaunchConfig
    from roslaunch.core import RLException
    from roslaunch.xmlloader import XmlLoader
    loader = XmlLoader()
    config = ROSLaunchConfig()
    test_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'xml'))
    try:
        loader.load(os.path.join(test_path, 'test-substitution-duplicate-anon-names.xml'), config)
        assert False, 'loading a launch file with duplicate anon node names should have raised an exception'
    except RLException:
        pass
