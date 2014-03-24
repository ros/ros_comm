# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#
# Revision $Id: substitution_args.py 15178 2011-10-10 21:22:53Z kwc $

"""
Library for processing XML substitution args. This is currently used
by roslaunch and xacro, but it is not yet a top-level ROS feature.
"""

import os

try:
    from cStringIO import StringIO # Python 2.x
except ImportError:
    from io import StringIO # Python 3.x

import rosgraph.names
import rospkg

_rospack = None

class SubstitutionException(Exception):
    """
    Base class for exceptions in substitution_args routines
    """
    pass
class ArgException(SubstitutionException):
    """
    Exception for missing $(arg) values
    """
    pass

def _env(resolved, a, args, context):
    """
    process $(env) arg
    @return: updated resolved argument
    @rtype: str
    @raise SubstitutionException: if arg invalidly specified
    """
    if len(args) != 1:
        raise SubstitutionException("$(env var) command only accepts one argument [%s]"%a)
    try:
        return resolved.replace("$(%s)"%a, os.environ[args[0]])
    except KeyError as e:
        raise SubstitutionException("environment variable %s is not set"%str(e))

def _optenv(resolved, a, args, context):
    """
    process $(optenv) arg
    @return: updated resolved argument
    @rtype: str
    @raise SubstitutionException: if arg invalidly specified
    """
    if len(args) == 0:
        raise SubstitutionException("$(optenv var) must specify an environment variable [%s]"%a)
    if args[0] in os.environ:
        return resolved.replace("$(%s)"%a, os.environ[args[0]])
    elif len(args) > 1:
        return resolved.replace("$(%s)"%a, ' '.join(args[1:]))
    else:
        return resolved.replace("$(%s)"%a, '')
    
def _anon(resolved, a, args, context):
    """
    process $(anon) arg
    @return: updated resolved argument
    @rtype: str
    @raise SubstitutionException: if arg invalidly specified
    """
    # #1559 #1660
    if len(args) == 0:
        raise SubstitutionException("$(anon var) must specify a name [%s]"%a)
    elif len(args) > 1:
        raise SubstitutionException("$(anon var) may only specify one name [%s]"%a)
    id = args[0]
    if 'anon' not in context:
        context['anon'] = {}
    anon_context = context['anon']
    if id in anon_context:
        return resolved.replace("$(%s)"%a, anon_context[id])
    else:
        resolve_to = rosgraph.names.anonymous_name(id)
        anon_context[id] = resolve_to
        return resolved.replace("$(%s)"%a, resolve_to)


def _find(resolved, a, args, context):
    """
    process $(find PKG)
    Resolves the path while considering the path following the command to provide backward compatible results.
    If it is followed by a path it first tries to resolve it as an executable and than as a normal file under share.
    Else it resolves to the source share folder of the PKG.
    :returns: updated resolved argument, ``str``
    :raises: :exc:SubstitutionException: if PKG invalidly specified
    :raises: :exc:`rospkg.ResourceNotFound` If PKG requires resource (e.g. package) that does not exist
    """
    if len(args) != 1:
        raise SubstitutionException("$(find pkg) command only accepts one argument [%s]" % a)
    before, after = _split_command(resolved, a)
    path, after = _separate_first_path(after)
    resolve_without_path = before + ('$(%s)' % a) + after
    path = _sanitize_path(path)
    if path.startswith('/') or path.startswith('\\'):
        path = path[1:]
    if path:
        try:
            return _find_executable(resolve_without_path, a, [args[0], path], context)
        except SubstitutionException:
            pass
        try:
            return _find_resource(resolve_without_path, a, [args[0], path], context)
        except SubstitutionException:
            pass
    rp = _get_rospack()
    pkg_path = rp.get_path(args[0])
    if path:
        pkg_path = os.path.join(pkg_path, path)
    return before + pkg_path + after


def _find_executable(resolved, a, args, _context):
    """
    process $(find-executable PKG PATH)
    It finds the executable with the basename(PATH) in the libexec folder
    or under the PATH relative to the package.xml file.
    :returns: updated resolved argument, ``str``
    :raises: :exc:SubstitutionException: if PKG/PATH invalidly specified or executable is not found for PKG
    """
    if len(args) != 2:
        raise SubstitutionException("$(find-executable pkg path) command only accepts two argument [%s]" % a)
    before, after = _split_command(resolved, a)
    path = _sanitize_path(args[1])
    # we try to find the specific executable in libexec via catkin
    # which will search in install/devel space
    full_path = None
    from catkin.find_in_workspaces import find_in_workspaces
    paths = find_in_workspaces(['libexec'], project=args[0], first_matching_workspace_only=True)  # implicitly first_match_only=True
    if paths:
        full_path = _get_executable_path(paths[0], os.path.basename(path))
    if not full_path:
        # else we will look for the executable in the source folder of the package
        rp = _get_rospack()
        full_path = _get_executable_path(rp.get_path(args[0]), path)
    if not full_path:
        raise SubstitutionException("$(find-executable pkg path) could not find executable [%s]" % a)
    return before + full_path + after


def _find_resource(resolved, a, args, _context):
    """
    process $(find-resource PKG PATH)
    Resolves the relative PATH from the share folder of the PKG either from install space, devel space or from the source folder.
    :returns: updated resolved argument, ``str``
    :raises: :exc:SubstitutionException: if PKG and PATH invalidly specified or relative PATH is not found for PKG
    """
    if len(args) != 2:
        raise SubstitutionException("$(find-resource pkg path) command only accepts two argument [%s]" % a)
    before, after = _split_command(resolved, a)
    path = _sanitize_path(args[1])
    # we try to find the specific path in share via catkin
    # which will search in install/devel space and the source folder of the package
    from catkin.find_in_workspaces import find_in_workspaces
    paths = find_in_workspaces(['share'], project=args[0], path=path, first_matching_workspace_only=True, first_match_only=True)
    if not paths:
        raise SubstitutionException("$(find-resource pkg path) could not find path [%s]" % a)
    return before + paths[0] + after


def _split_command(resolved, command_with_args):
    cmd = '$(%s)' % command_with_args
    idx1 = resolved.find(cmd)
    idx2 = idx1 + len(cmd)
    return resolved[0:idx1], resolved[idx2:]


def _separate_first_path(value):
    idx = value.find(' ')
    if idx < 0:
        path, rest = value, ''
    else:
        path, rest = value[0:idx], value[idx:]
    return path, rest


def _sanitize_path(path):
    path = path.replace('/', os.sep)
    path = path.replace('\\', os.sep)
    return path


def _get_executable_path(base_path, path):
    full_path = os.path.join(base_path, path)
    if os.path.isfile(full_path) and os.access(full_path, os.X_OK):
        return full_path
    return None


def _get_rospack():
    global _rospack
    if _rospack is None:
        _rospack = rospkg.RosPack()
    return _rospack


def _arg(resolved, a, args, context):
    """
    process $(arg) arg
    
    :returns: updated resolved argument, ``str``
    :raises: :exc:`ArgException` If arg invalidly specified
    """
    if len(args) == 0:
        raise SubstitutionException("$(arg var) must specify an <arg> tag name [%s]"%(a))
    elif len(args) > 1:
        raise SubstitutionException("$(arg var) may only specify one arg [%s]"%(a))
    
    if 'arg' not in context:
        context['arg'] = {}
    arg_context = context['arg']

    arg_name = args[0]
    if arg_name in arg_context:
        arg_value = arg_context[arg_name]
        return resolved.replace("$(%s)"%a, arg_value)
    else:
        raise ArgException(arg_name)


def resolve_args(arg_str, context=None, resolve_anon=True):
    """
    Resolves substitution args (see wiki spec U{http://ros.org/wiki/roslaunch}).

    @param arg_str: string to resolve zero or more substitution args
        in. arg_str may be None, in which case resolve_args will
        return None
    @type  arg_str: str
    @param context dict: (optional) dictionary for storing results of
        the 'anon' and 'arg' substitution args. multiple calls to
        resolve_args should use the same context so that 'anon'
        substitions resolve consistently. If no context is provided, a
        new one will be created for each call. Values for the 'arg'
        context should be stored as a dictionary in the 'arg' key.
    @type  context: dict
    @param resolve_anon bool: If True (default), will resolve $(anon
        foo). If false, will leave these args as-is.
    @type  resolve_anon: bool

    @return str: arg_str with substitution args resolved
    @rtype:  str
    @raise SubstitutionException: if there is an error resolving substitution args
    """
    if context is None:
        context = {}
    #parse found substitution args
    if not arg_str:
        return arg_str
    # first resolve variables like 'env' and 'arg'
    commands = {
        'env': _env,
        'optenv': _optenv,
        'anon': _anon,
        'arg': _arg,
    }
    resolved = _resolve_args(arg_str, context, resolve_anon, commands)
    # than resolve 'find' as it requires the subsequent path to be expanded already
    commands = {
        'find': _find,
    }
    resolved = _resolve_args(resolved, context, resolve_anon, commands)
    return resolved

def _resolve_args(arg_str, context, resolve_anon, commands):
    valid = ['find', 'env', 'optenv', 'anon', 'arg']
    resolved = arg_str
    for a in _collect_args(arg_str):
        splits = [s for s in a.split(' ') if s]
        if not splits[0] in valid:
            raise SubstitutionException("Unknown substitution command [%s]. Valid commands are %s"%(a, valid))
        command = splits[0]
        args = splits[1:]
        if command in commands:
            resolved = commands[command](resolved, a, args, context)
    return resolved

_OUT  = 0
_DOLLAR = 1
_LP = 2
_IN = 3
def _collect_args(arg_str):
    """
    State-machine parser for resolve_args. Substitution args are of the form:
    $(find package_name)/scripts/foo.py $(export some/attribute blar) non-relevant stuff
    
    @param arg_str: argument string to parse args from
    @type  arg_str: str
    @raise SubstitutionException: if args are invalidly specified
    @return: list of arguments
    @rtype: [str]
    """
    buff = StringIO()
    args = []
    state = _OUT
    for c in arg_str:
        # No escapes supported
        if c == '$':
            if state == _OUT:
                state = _DOLLAR
            elif state == _DOLLAR:
                pass
            else:
                raise SubstitutionException("Dollar signs '$' cannot be inside of substitution args [%s]"%arg_str)
        elif c == '(':
            if state == _DOLLAR:
                state = _LP
            elif state != _OUT:
                raise SubstitutionException("Invalid left parenthesis '(' in substitution args [%s]"%arg_str)
        elif c == ')':
            if state == _IN:
                #save contents of collected buffer
                args.append(buff.getvalue())
                buff.truncate(0)
                buff.seek(0)
                state = _OUT
            else:
                state = _OUT
        elif state == _DOLLAR:
            # left paren must immediately follow dollar sign to enter _IN state
            state = _OUT
        elif state == _LP:
            state = _IN

        if state == _IN:
            buff.write(c)
    return args


