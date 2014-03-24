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
import re

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

def _env(cmd, args, context):
    """
    process $(env) arg
    @return: updated resolved argument
    @rtype: str
    @raise SubstitutionException: if arg invalidly specified
    """
    if len(args) != 1:
        raise SubstitutionException("$(env var) command only accepts one argument [%s]"%a)
    try:
        return os.environ[args[0]]
    except KeyError as e:
        raise SubstitutionException("environment variable %s is not set"%str(e))

def _optenv(cmd, args, context):
    """
    process $(optenv) arg
    @return: updated resolved argument
    @rtype: str
    @raise SubstitutionException: if arg invalidly specified
    """
    if len(args) == 0:
        raise SubstitutionException("$(optenv var) must specify an environment variable [%s]"%cmd)
    if args[0] in os.environ:
        return os.environ[args[0]]
    elif len(args) > 1:
        return ' '.join(args[1:])
    else:
        return ''

def _anon(cmd, args, context):
    """
    process $(anon) arg
    @return: updated resolved argument
    @rtype: str
    @raise SubstitutionException: if arg invalidly specified
    """
    # #1559 #1660
    if len(args) == 0:
        raise SubstitutionException("$(anon var) must specify a name [%s]"%cmd)
    elif len(args) > 1:
        raise SubstitutionException("$(anon var) may only specify one name [%s]"%cmd)
    id = args[0]
    if 'anon' not in context:
        context['anon'] = {}
    anon_context = context['anon']
    if id in anon_context:
        return anon_context[id]
    else:
        resolve_to = rosgraph.names.anonymous_name(id)
        anon_context[id] = resolve_to
        return resolve_to


def _find(resolved, a, args, context):
    """
    process $(find PKG)
    Resolves the path while considering the path following the command to provide backward compatible results.
    If it is followed by a path it first tries to resolve it as an executable and than as a normal file under share.
    Else it resolves to the source share folder of the PKG.

    :param resolved: full string
    :type resolved: str
    :param a: subtitution string
    :type a: str
    :param args: subtitution arguments
    :type a: dict
    :param context: see resolve_args()
    :type context: dict

    :returns: updated resolved argument, ``str``
    :raises: :exc:SubstitutionException: if PKG invalidly specified
    :raises: :exc:`rospkg.ResourceNotFound` If PKG requires resource (e.g. package) that does not exist
    """
    assert len(args) == 1

    def _separate_first_path(value):
        idx = value.find(' ')
        if idx < 0:
            path, rest = value, ''
        else:
            path, rest = value[0:idx], value[idx:]
        return path, rest

    # These variables will contain:
    #  * before: everything before the $()
    #  * path: everything after $() up until the first space
    #  * after: whatever remains after `path'
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
    assert len(args) == 2
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
    assert len(args) == 2
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
    """ Given a string 'a $(b) c', where b == command_with_args, returns (a, c). """
    cmd = '$(%s)' % command_with_args
    idx1 = resolved.find(cmd)
    idx2 = idx1 + len(cmd)
    return resolved[0:idx1], resolved[idx2:]


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


def _arg(cmd, args, context):
    """
    process $(arg) arg
    
    :returns: updated resolved argument, ``str``
    :raises: :exc:`ArgException` If arg invalidly specified
    """
    if len(args) == 0:
        raise SubstitutionException("$(arg var) must specify an <arg> tag name [%s]"%cmd)
    elif len(args) > 1:
        raise SubstitutionException("$(arg var) may only specify one arg [%s]"%cmd)

    if 'arg' not in context:
        context['arg'] = {}
    arg_context = context['arg']

    arg_name = args[0]
    if arg_name in arg_context:
        return arg_context[arg_name]
    else:
        raise ArgException(arg_name)


# TODO: resolve_anon is being ignored completely! Can it be removed?
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
    if not arg_str:
        return arg_str

    root = _parse_args(arg_str)
    string = _apply_substitutions(root, context, _ParseTree.valid_commands)

    # Hack for $find(), since it requires the full string for its magic.
    for subst in _FIND_REGEX.findall(string):
        subst = subst[2:-1]
        splits = [s for s in subst.split(' ') if s]
        string = _find(string, subst, splits[1:], context)

    return string

_FIND_REGEX = re.compile(r'\$\(find .*?\)');
def _find_hack(cmd, args, context):
    """
    _find() requires the full string (including stuff outside the "$(...)"), so it is handled
    separately.

    :returns: original text (unresolved)
    :raises: :exc:SubstitutionException: if PKG invalidly specified
    """
    if len(args) != 1:
        raise SubstitutionException("$(find pkg) command only accepts one argument [%s]" % cmd)
    return '$(%s)' % cmd


class _ParseTree:
    """
    Parse tree (auxiliary data structure, returned by _parse_args).

    Attributes:
     * parent: the parent _ParseTree instance (None for the root node)
     * childs: ordered list of strings and _ParseTree instances

    Note: except for the root node, the first child is always a string
          where the first word identifies the substitution arg.
    """

    valid_commands = {
        'anon': _anon,
        'arg': _arg,
        'env': _env,
        'find': _find_hack,
        'optenv': _optenv,
    }

    def __init__(self, parent=None):
        self.parent = parent
        self.childs = []

    def _add_child(self, child):
        """
        Adds an element to the list of childs. If called with an empty string,
        nothing happens.

        @param child: element to add to the childs list
        @type child: str or L{_ParseTree}
        @raise SubstitutionException: if the first child of a non-root node isn't
               a valid substitution command.
        """
        if child:
            if len(self.childs) == 0 and not self.is_root():
                cmd = child.split(' ', 1)[0]
                if cmd not in self.valid_commands:
                    raise SubstitutionException(
                        "Unknown substitution command [%s]. Valid commands are %s" % (
                        child, self.valid_commands.keys()))
            self.childs.append(child)
        return child

    def is_root(self):
        """
        Returns true if this is the root node (ie. it has no parent).

        @return whether this is a root node
        @rtype bool
        """
        return self.parent is None

    def __repr__(self):
        child_strs = [('%r' % c if isinstance(c, _ParseTree) else '"%s"' % c) for c in self.childs]
        return 'Tree(%s)' % ', '.join(child_strs)


def _apply_substitutions(node, context, commands):
    """
    Takes a parse tree and converts it into a flat string, calling the appropriate
    function for each substitution (non-root node).

    param node: the parse tree to convert into a string
    @type node: L{_ParseTree}
    @param context See resolve_args()
    @type context: dict
    @param commands: dictionary with (command name -> callable object) mappings
    @type commands: dict

    @return string resulting after applying all substitutions
    @rtype str
    @raise SubstitutionException: if there is an error resolving substitution args
    """
    resolved = []
    for child in node.childs:
        if not isinstance(child, _ParseTree):
            resolved.append(child)
        else:
            s = _apply_substitutions(child, context, commands)
            splits = [w for w in s.split(' ') if w]
            resolved.append(commands[splits[0]](s, splits[1:], context))
    return ''.join(resolved)


def _parse_args(arg_str):
    """
    Parser for substitutions, with support for nested expressions. It parses strings
    of the form:
      '$(find package_name)/scripts/foo.py $(export some/attribute blar) non-relevant stuff'
      '$(find $(env ROBOT_NAME)_maps)'

    @param arg_str: argument string to parse args from
    @type  arg_str: str
    @raise SubstitutionException: if args are invalidly specified
    @return: root node of the parse tree
    @rtype: L{_ParseTree}
    """
    i = 0  # pointer to the left-most character that isn't in the tree
    j = 0  # pointer to the character being evaluated
    t = _ParseTree()
    while j < len(arg_str):
        if arg_str[j] == '$' and arg_str[j+1] == '(':
            t._add_child(arg_str[i:j])
            # Starting a new substitution
            t = t._add_child(_ParseTree(t))
            i = j = j + 2
        elif arg_str[j] == ')' and not t.is_root():
            t._add_child(arg_str[i:j])
            # End of the current substituion
            t = t.parent
            i = j = j + 1
        else:
            # Checks that were present in the previous parser (without nesting support)
            if arg_str[j] == '(' and not t.is_root():
                raise SubstitutionException("Invalid left parenthesis '(' in substitution args [%s]" % arg_str)
            if arg_str[j] == '$' and not t.is_root():
                raise SubstitutionException("Dollar signs '$' cannot be inside of substitution args [%s]" % arg_str)
            j += 1
    # Add remaining text content...
    t._add_child(arg_str[i:j])

    if not t.is_root():
        raise SubstitutionException("Unbalanced parentheses in substitution args [%s]" % arg_str)
    return t
