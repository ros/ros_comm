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
    arg_context = {'fuga': 'hoge', 'car': 'cdr'}
    context = {'anon': anon_context, 'arg': arg_context }

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

        # #1776
        ('$(anon foo)', 'bar'),
        ('$(anon foo)/baz', 'bar/baz'),
        ('$(anon foo)/baz/$(anon foo)', 'bar/baz/bar'),

        # arg
        ('$(arg fuga)', 'hoge'),
        ('$(arg fuga)$(arg fuga)', 'hogehoge'),
        ('$(arg car)$(arg fuga)', 'cdrhoge'),
        ('$(arg fuga)hoge', 'hogehoge'),
        ]
    for arg, val in tests:
        assert val == resolve_args(arg, context=context)

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


def test__eq():
    import random
    from roslaunch.substitution_args import _eq, ArgException, SubstitutionException

    ctx = {
        'arg': {
            'foo': '12345',
            'bar': 'hello world',
            'baz': 'sim',
            'empty': '',
        }
    }

    # test a missing arg and value
    try:
        inside = '$(eq)'
        _eq('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test a missing value
    try:
        inside = 'eq arg'
        _eq('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test an unknown argument name
    try:
        inside = "eq missing_arg value"
        _eq('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test normal
    tests = [
        ('true', ('$(eq foo 12345)', 'eq foo 12345', ['foo', '12345'], ctx)),
        ('false', ('$(eq foo 543221)', 'eq foo 54321', ['foo', '54321'], ctx)),
        ('true', ('$(eq baz sim)', 'eq baz sim', ['baz', 'sim'], ctx)),

        # test with other args present, should only resolve match
        ('true', ('$(eq foo 12345)$(eq foo 54321)', 'eq foo 12345', ['foo', '12345'], ctx)),
        ('false', ('$(eq foo 12345)$(eq baz 54321)', 'eq foo 54321', ['foo', '54321'], ctx)),
        ]

    for result, test in tests:
        resolved, a, args, context = test
        assert result == _eq(resolved, a, args, context)

    #  - test that all fail if ctx is not set
    for result, test in tests:
        resolved, a, args, context = test
        try:
            _eq(resolved, a, args, {})
            assert False, "should have thrown"
        except ArgException as e:
            assert args[0] == str(e)
        try:
            _eq(resolved, a, args, {'arg': {}})
            assert False, "should have thrown"
        except ArgException as e:
            assert args[0] == str(e)


def test__neq():
    import random
    from roslaunch.substitution_args import _neq, ArgException, SubstitutionException

    ctx = {
        'arg': {
            'foo': '12345',
            'bar': 'hello world',
            'baz': 'sim',
            'empty': '',
        }
    }

    # test a missing arg and value
    try:
        inside = '$(neq)'
        _neq('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test a missing value
    try:
        inside = 'neq arg'
        _neq('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test an unknown argument name
    try:
        inside = "neq missing_arg value"
        _neq('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test normal
    tests = [
        ('false', ('$(neq foo 12345)', 'neq foo 12345', ['foo', '12345'], ctx)),
        ('true', ('$(neq foo 543221)', 'neq foo 54321', ['foo', '54321'], ctx)),
        ('false', ('$(neq baz sim)', 'neq baz sim', ['baz', 'sim'], ctx)),

        # test with other args present, should only resolve match
        ('false', ('$(neq foo 12345)$(neq foo 54321)', 'neq foo 12345', ['foo', '12345'], ctx)),
        ('true', ('$(neq foo 12345)$(neq baz 54321)', 'neq foo 54321', ['foo', '54321'], ctx)),
        ]

    for result, test in tests:
        resolved, a, args, context = test
        assert result == _neq(resolved, a, args, context)

    #  - test that all fail if ctx is not set
    for result, test in tests:
        resolved, a, args, context = test
        try:
            _neq(resolved, a, args, {})
            assert False, "should have thrown"
        except ArgException as e:
            assert args[0] == str(e)
        try:
            _neq(resolved, a, args, {'arg': {}})
            assert False, "should have thrown"
        except ArgException as e:
            assert args[0] == str(e)


def test__eqarg():
    import random
    from roslaunch.substitution_args import _eqarg, ArgException, SubstitutionException

    ctx = {
        'arg': {
            'foo': '12345',
            'bar': 'hello world',
            'baz': 'sim',
            'empty': '',
            'foo2': '12345',
        }
    }

    # test a missing arg and value
    try:
        inside = '$(eqarg)'
        _eqarg('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test a missing value
    try:
        inside = 'eqarg arg'
        _eqarg('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test an unknown argument name
    try:
        inside = "eqarg missing_arg value"
        _eqarg('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test an unknown value name
    try:
        inside = "eqarg foo missing_arg"
        _eqarg('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test normal
    tests = [
        ('true', ('$(eqarg foo foo)', 'eqarg foo foo', ['foo', 'foo'], ctx)),
        ('false', ('$(eqarg foo bar)', 'eqarg foo bar', ['foo', 'bar'], ctx)),
        ('true', ('$(eqarg foo foo2)', 'eqarg foo foo2', ['foo', 'foo2'], ctx)),
        ('false', ('$(eqarg baz empty)', 'eqarg baz empty', ['baz', 'empty'], ctx)),

        # test with other args present, should only resolve match
        ('false', ('$(eqarg foo bar)$(eqarg foo foo2)', 'eqarg foo bar', ['foo', 'bar'], ctx)),
        ('true', ('$(eqarg foo baz)$(eqarg foo2 foo)', 'eqarg foo2 foo', ['foo2', 'foo'], ctx)),
        ]

    for result, test in tests:
        resolved, a, args, context = test
        assert result == _eqarg(resolved, a, args, context)

    #  - test that all fail if ctx is not set
    for result, test in tests:
        resolved, a, args, context = test
        try:
            _eqarg(resolved, a, args, {})
            assert False, "should have thrown"
        except ArgException as e:
            assert args[0] == str(e)
        try:
            _eqarg(resolved, a, args, {'arg': {}})
            assert False, "should have thrown"
        except ArgException as e:
            assert args[0] == str(e)


def test__neqarg():
    import random
    from roslaunch.substitution_args import _neqarg, ArgException, SubstitutionException

    ctx = {
        'arg': {
            'foo': '12345',
            'bar': 'hello world',
            'baz': 'sim',
            'empty': '',
            'foo2': '12345',
        }
    }

    # test a missing arg and value
    try:
        inside = '$(neqarg)'
        _neqarg('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test a missing value
    try:
        inside = 'neqarg arg'
        _neqarg('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test an unknown argument name
    try:
        inside = "neqarg missing_arg value"
        _neqarg('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test an unknown value name
    try:
        inside = "neqarg foo missing_arg"
        _neqarg('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test normal
    tests = [
        ('false', ('$(neqarg foo foo)', 'neqarg foo foo', ['foo', 'foo'], ctx)),
        ('true', ('$(neqarg foo bar)', 'neqarg foo bar', ['foo', 'bar'], ctx)),
        ('false', ('$(neqarg foo foo2)', 'neqarg foo foo2', ['foo', 'foo2'], ctx)),
        ('true', ('$(neqarg baz empty)', 'neqarg baz empty', ['baz', 'empty'], ctx)),

        # test with other args present, should only resolve match
        ('true', ('$(neqarg foo bar)$(neqarg foo foo2)', 'neqarg foo bar', ['foo', 'bar'], ctx)),
        ('false', ('$(neqarg foo baz)$(neqarg foo2 foo)', 'neqarg foo2 foo', ['foo2', 'foo'], ctx)),
        ]

    for result, test in tests:
        resolved, a, args, context = test
        assert result == _neqarg(resolved, a, args, context)

    #  - test that all fail if ctx is not set
    for result, test in tests:
        resolved, a, args, context = test
        try:
            _neqarg(resolved, a, args, {})
            assert False, "should have thrown"
        except ArgException as e:
            assert args[0] == str(e)
        try:
            _neqarg(resolved, a, args, {'arg': {}})
            assert False, "should have thrown"
        except ArgException as e:
            assert args[0] == str(e)


def test__empty():
    import random
    from roslaunch.substitution_args import _empty, ArgException, SubstitutionException

    ctx = {
        'arg': {
            'foo': '12345',
            'bar': 'hello world',
            'baz': 'sim',
            'empty': '',
            'bang': '',
        }
    }

    # test a missing arg
    try:
        inside = '$(empty)'
        _empty('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test an unknown argument name
    try:
        inside = "empty missing_arg"
        _empty('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test normal
    tests = [
        ('false', ('$(empty foo)', 'empty foo', ['foo'], ctx)),
        ('true', ('$(empty empty)', 'empty empty', ['empty'], ctx)),
        ('true', ('$(empty bang)', 'empty bang', ['bang'], ctx)),
        ('false', ('$(empty bar)', 'empty bar', ['bar'], ctx)),

        # test with other args present, should only resolve match
        ('true', ('$(empty bang)$(empty foo)', 'empty bang', ['bang'], ctx)),
        ('false', ('$(empty bang)$(empty bar)', 'empty bar', ['bar'], ctx)),
        ]

    for result, test in tests:
        resolved, a, args, context = test
        assert result == _empty(resolved, a, args, context)

    #  - test that all fail if ctx is not set
    for result, test in tests:
        resolved, a, args, context = test
        try:
            _empty(resolved, a, args, {})
            assert False, "should have thrown"
        except ArgException as e:
            assert args[0] == str(e)
        try:
            _empty(resolved, a, args, {'arg': {}})
            assert False, "should have thrown"
        except ArgException as e:
            assert args[0] == str(e)


def test__notempty():
    import random
    from roslaunch.substitution_args import _notempty, ArgException, SubstitutionException

    ctx = {
        'arg': {
            'foo': '12345',
            'bar': 'hello world',
            'baz': 'sim',
            'notempty': '',
            'bang': '',
        }
    }

    # test a missing arg
    try:
        inside = '$(notempty)'
        _notempty('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test an unknown argument name
    try:
        inside = "notempty missing_arg"
        _notempty('$(%s)' % inside, inside, [], ctx)
        assert False, "should have thrown"
    except SubstitutionException:
        pass

    # test normal
    tests = [
        ('true', ('$(notempty foo)', 'notempty foo', ['foo'], ctx)),
        ('false', ('$(notempty notempty)', 'notempty notempty', ['notempty'], ctx)),
        ('false', ('$(notempty bang)', 'notempty bang', ['bang'], ctx)),
        ('true', ('$(notempty bar)', 'notempty bar', ['bar'], ctx)),

        # test with other args present, should only resolve match
        ('false', ('$(notempty bang)$(notempty foo)', 'notempty bang', ['bang'], ctx)),
        ('true', ('$(notempty bang)$(notempty bar)', 'notempty bar', ['bar'], ctx)),
        ]

    for result, test in tests:
        resolved, a, args, context = test
        assert result == _notempty(resolved, a, args, context)

    #  - test that all fail if ctx is not set
    for result, test in tests:
        resolved, a, args, context = test
        try:
            _notempty(resolved, a, args, {})
            assert False, "should have thrown"
        except ArgException as e:
            assert args[0] == str(e)
        try:
            _notempty(resolved, a, args, {'arg': {}})
            assert False, "should have thrown"
        except ArgException as e:
            assert args[0] == str(e)
