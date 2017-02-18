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
# Revision $Id$

"""
Roslaunch XML file parser.
"""

from __future__ import print_function

import itertools
import sys
import traceback
import logging

from xml.dom.minidom import parse, parseString
from xml.dom import Node as DomNode #avoid aliasing

from rosgraph.names import make_global_ns, ns_join, is_private, is_legal_name, get_ros_namespace

from .core import Param, Node, Test, Machine, RLException
from . import loader
from . import substitution_args

# use in our namespace
SubstitutionException = substitution_args.SubstitutionException
ArgException = substitution_args.ArgException

NS='ns'
CLEAR_PARAMS='clear_params'

def _get_text(tag):
    buff = ''
    for t in tag.childNodes:
        if t.nodeType in [t.TEXT_NODE, t.CDATA_SECTION_NODE]:
            buff += t.data
    return buff

def ifunless_test(obj, tag, context):
    """
    @return True: if tag should be processed according to its if/unless attributes
    """
    if_val, unless_val = obj.opt_attrs(tag, context, ['if', 'unless'])
    if if_val is not None and unless_val is not None:
        raise XmlParseException("cannot set both 'if' and 'unless' on the same tag")
    if if_val is not None:
        if_val = loader.convert_value(if_val, 'bool')
        if if_val:
            return True
    elif unless_val is not None:
        unless_val = loader.convert_value(unless_val, 'bool')
        if not unless_val:
            return True
    else:
        return True
    return False
    
def ifunless(f):
    """
    Decorator for evaluating whether or not tag function should run based on if/unless attributes
    """
    def call(*args, **kwds):
        #TODO: logging, as well as check for verbose in kwds
        if ifunless_test(args[0], args[1], args[2]):
            return f(*args, **kwds)
    return call

# This code has gotten a bit crufty as roslaunch has grown far beyond
# its original spec. It needs to be far more generic than it is in
# order to not replicate bugs in multiple places.

class XmlParseException(RLException):
    """Error with the XML syntax (e.g. invalid attribute/value combinations)"""
    pass

def _bool_attr(v, default, label):
    """
    Validate boolean xml attribute. 
    @param v: parameter value or None if no value provided
    @type v: any
    @param default: default value
    @type  default: bool
    @param label: parameter name/label
    @type  label: str
    @return: boolean value for attribute
    @rtype: bool
    @raise XmlParseException: if v is not in correct range or is empty.
    """
    if v is None:
        return default
    if v.lower() == 'true':
        return True
    elif v.lower() == 'false':
        return False
    elif not v:
        raise XmlParseException("bool value for %s must be non-empty"%(label))
    else:
        raise XmlParseException("invalid bool value for %s: %s"%(label, v))

def _float_attr(v, default, label):
    """
    Validate float xml attribute.
    @param v: parameter value or None if no value provided
    @type v: any
    @param default: default value
    @type  default: float
    @param label: parameter name/label
    @type  label: str
    @return: float value for attribute
    @rtype: float
    @raise XmlParseException: if v is not in correct range or is empty.
    """
    if v is None:
        return default
    if not v:
        raise XmlParseException("bool value for %s must be non-empty"%(label))
    try:
        x = float(v)
    except ValueError:
        raise XmlParseException("invalid float value for %s: %s"%(label, v))
    return x


# maps machine 'default' attribute to Machine default property
_is_default = {'true': True, 'false': False, 'never': False }
# maps machine 'default' attribute to Machine assignable property
_assignable = {'true': True, 'false': True, 'never': False }

# NOTE: code is currently in a semi-refactored state. I'm slowly
# migrating common routines into the Loader class in the hopes it will
# make it easier to write alternate loaders and also test.
class XmlLoader(loader.Loader):
    """
    Parser for roslaunch XML format. Loads parsed representation into ROSConfig model.
    """

    def __init__(self, resolve_anon=True):
        """
        @param resolve_anon: If True (default), will resolve $(anon foo). If
        false, will leave these args as-is.
        @type  resolve_anon: bool
        """        
        # store the root XmlContext so that outside code can access it
        self.root_context = None
        self.resolve_anon = resolve_anon

    def resolve_args(self, args, context):
        """
        Wrapper around substitution_args.resolve_args to set common parameters
        """
        # resolve_args gets called a lot, so we optimize by testing for dollar sign before resolving
        if args and '$' in args:
            return substitution_args.resolve_args(args, context=context.resolve_dict, resolve_anon=self.resolve_anon)
        else:
            return args

    def opt_attrs(self, tag, context, attrs):
        """
        Helper routine for fetching and resolving optional tag attributes
        @param tag DOM tag
        @param context LoaderContext
        @param attrs (str): list of attributes to resolve
        """            
        def tag_value(tag, a):
            if tag.hasAttribute(a):
                # getAttribute returns empty string for non-existent
                # attributes, which makes it impossible to distinguish
                # with actual empty values
                return tag.getAttribute(a)
            else:
                return None
        return [self.resolve_args(tag_value(tag,a), context) for a in attrs]

    def reqd_attrs(self, tag, context, attrs):
        """
        Helper routine for fetching and resolving required tag attributes
        @param tag: DOM tag
        @param attrs: list of attributes to resolve        
        @type  attrs: (str)
        @raise KeyError: if required attribute is missing
        """            
        return [self.resolve_args(tag.attributes[a].value, context) for a in attrs]

    def _check_attrs(self, tag, context, ros_config, attrs):
        tag_attrs = tag.attributes.keys()
        for t_a in tag_attrs:
            if not t_a in attrs and not t_a in ['if', 'unless']:
                ros_config.add_config_error("[%s] unknown <%s> attribute '%s'"%(context.filename, tag.tagName, t_a))

    # 'ns' attribute is now deprecated and is an alias for
    # 'param'. 'param' is required if the value is a non-dictionary
    # type
    ROSPARAM_OPT_ATTRS = ('command', 'ns', 'file', 'param', 'subst_value')
    @ifunless
    def _rosparam_tag(self, tag, context, ros_config, verbose=True):
        try:
            self._check_attrs(tag, context, ros_config, XmlLoader.ROSPARAM_OPT_ATTRS)
            cmd, ns, file, param, subst_value = self.opt_attrs(tag, context, (XmlLoader.ROSPARAM_OPT_ATTRS))
            subst_value = _bool_attr(subst_value, False, 'subst_value')
            # ns atribute is a bit out-moded and is only left in for backwards compatibility
            param = ns_join(ns or '', param or '')
            
            # load is the default command            
            cmd = cmd or 'load'
            value = _get_text(tag)
            if subst_value:
                value = self.resolve_args(value, context)
            self.load_rosparam(context, ros_config, cmd, param, file, value, verbose=verbose)

        except ValueError as e:
            raise loader.LoadException("error loading <rosparam> tag: \n\t"+str(e)+"\nXML is %s"%tag.toxml())

    PARAM_ATTRS = ('name', 'value', 'type', 'value', 'textfile', 'binfile', 'command')
    @ifunless
    def _param_tag(self, tag, context, ros_config, force_local=False, verbose=True):
        """
        @param force_local: if True, param must be added to context instead of ros_config
        @type  force_local: bool
        """
        try:
            self._check_attrs(tag, context, ros_config, XmlLoader.PARAM_ATTRS)

            # compute name and value
            ptype = (tag.getAttribute('type') or 'auto').lower().strip()
            
            vals = self.opt_attrs(tag, context, ('value', 'textfile', 'binfile', 'command'))
            if len([v for v in vals if v is not None]) != 1:
                raise XmlParseException(
                    "<param> tag must have one and only one of value/textfile/binfile.")

            # compute name. if name is a tilde name, it is placed in
            # the context. otherwise it is placed in the ros config.
            name = self.resolve_args(tag.attributes['name'].value.strip(), context)
            value = self.param_value(verbose, name, ptype, *vals)

            if is_private(name) or force_local:
                p = Param(name, value)
                context.add_param(p)
            else:
                p = Param(ns_join(context.ns, name), value)
                ros_config.add_param(Param(ns_join(context.ns, name), value), filename=context.filename, verbose=verbose)
            return p

        except KeyError as e:
            raise XmlParseException(
                "<param> tag is missing required attribute: %s. \n\nParam xml is %s"%(e, tag.toxml()))
        except ValueError as e:
            raise XmlParseException(
                "Invalid <param> tag: %s. \n\nParam xml is %s"%(e, tag.toxml()))

    ARG_ATTRS = ('name', 'value', 'default', 'doc')
    @ifunless
    def _arg_tag(self, tag, context, ros_config, verbose=True):
        """
        Process an <arg> tag.
        """
        try:
            self._check_attrs(tag, context, ros_config, XmlLoader.ARG_ATTRS)
            (name,) = self.reqd_attrs(tag, context, ('name',))
            value, default, doc = self.opt_attrs(tag, context, ('value', 'default', 'doc'))
            
            if value is not None and default is not None:
                raise XmlParseException(
                    "<arg> tag must have one and only one of value/default.")
            
            context.add_arg(name, value=value, default=default, doc=doc)

        except substitution_args.ArgException as e:
            raise XmlParseException(
                "arg '%s' is not defined. \n\nArg xml is %s"%(e, tag.toxml()))
        except Exception as e:
            raise XmlParseException(
                "Invalid <arg> tag: %s. \n\nArg xml is %s"%(e, tag.toxml()))

    def _test_attrs(self, tag, context):
        """
        Process attributes of <test> tag not present in <node>
        @return: test_name, time_limit
        @rtype: str, int
        """
        for attr in ['respawn', 'respawn_delay', 'output']:
            if tag.hasAttribute(attr):
                raise XmlParseException("<test> tags cannot have '%s' attribute"%attr)

        test_name = self.resolve_args(tag.attributes['test-name'].value, context)
        time_limit = self.resolve_args(tag.getAttribute('time-limit'), context)
        retry = self.resolve_args(tag.getAttribute('retry'), context)        
        if time_limit:
            try:
                time_limit = float(time_limit)
            except ValueError:
                raise XmlParseException("'time-limit' must be a number: [%s]"%time_limit)
            if time_limit <= 0.0:
                raise XmlParseException("'time-limit' must be a positive number")
        if retry:
            try:
                retry = int(retry)
            except ValueError:
                raise XmlParseException("'retry' must be a number: [%s]"%retry)

        return test_name, time_limit, retry
        
    NODE_ATTRS = ['pkg', 'type', 'machine', 'name', 'args', 'output', \
            'respawn', 'respawn_delay', 'cwd', NS, CLEAR_PARAMS, \
            'launch-prefix', 'required']
    TEST_ATTRS = NODE_ATTRS + ['test-name','time-limit', 'retry']
    
    @ifunless
    def _node_tag(self, tag, context, ros_config, default_machine, is_test=False, verbose=True):
        """
        Process XML <node> or <test> tag
        @param tag: DOM node
        @type  tag: Node
        @param context: namespace context
        @type  context: L{LoaderContext}
        @param params: ROS parameter list
        @type  params: [L{Param}]
        @param clear_params: list of ROS parameter names to clear before setting parameters
        @type  clear_params: [str]
        @param default_machine: default machine to assign to node
        @type  default_machine: str
        @param is_test: if set, will load as L{Test} object instead of L{Node} object
        @type  is_test: bool
        """
        try:
            if is_test:
                self._check_attrs(tag, context, ros_config, XmlLoader.TEST_ATTRS)
                (name,) = self.opt_attrs(tag, context, ('name',)) 
                test_name, time_limit, retry = self._test_attrs(tag, context)
                if not name:
                    name = test_name
            else:
                self._check_attrs(tag, context, ros_config, XmlLoader.NODE_ATTRS)
                (name,) = self.reqd_attrs(tag, context, ('name',))

            if not is_legal_name(name):
                ros_config.add_config_error("WARN: illegal <node> name '%s'.\nhttp://ros.org/wiki/Names\nThis will likely cause problems with other ROS tools.\nNode xml is %s"%(name, tag.toxml()))
                    
            child_ns = self._ns_clear_params_attr('node', tag, context, ros_config, node_name=name)
            param_ns = child_ns.child(name)
                
            # required attributes
            pkg, node_type = self.reqd_attrs(tag, context, ('pkg', 'type'))
            
            # optional attributes
            machine, args, output, respawn, respawn_delay, cwd, launch_prefix, \
                    required = self.opt_attrs(tag, context, ('machine', 'args',
                        'output', 'respawn', 'respawn_delay', 'cwd',
                        'launch-prefix', 'required'))
            if tag.hasAttribute('machine') and not len(machine.strip()):
                raise XmlParseException("<node> 'machine' must be non-empty: [%s]"%machine)
            if not machine and default_machine:
                machine = default_machine.name
            # validate respawn, required
            required, respawn = [_bool_attr(*rr) for rr in ((required, False, 'required'),\
                                                                (respawn, False, 'respawn'))]
            respawn_delay = _float_attr(respawn_delay, 0.0, 'respawn_delay')

            # each node gets its own copy of <remap> arguments, which
            # it inherits from its parent
            remap_context = context.child('')

            # each node gets its own copy of <env> arguments, which
            # it inherits from its parent
            env_context = context.child('')

            # nodes can have individual env args set in addition to
            # the ROS-specific ones.  
            for t in [c for c in tag.childNodes if c.nodeType == DomNode.ELEMENT_NODE]:
                tag_name = t.tagName.lower()
                if tag_name == 'remap':
                    r = self._remap_tag(t, context, ros_config)
                    if r is not None:
                        remap_context.add_remap(r)
                elif tag_name == 'param':
                    self._param_tag(t, param_ns, ros_config, force_local=True, verbose=verbose)
                elif tag_name == 'rosparam':
                    self._rosparam_tag(t, param_ns, ros_config, verbose=verbose)
                elif tag_name == 'env':
                    self._env_tag(t, env_context, ros_config)
                else:
                    ros_config.add_config_error("WARN: unrecognized '%s' child tag in the parent tag element: %s"%(t.tagName, tag.toxml()))

            # #1036 evaluate all ~params in context
            # TODO: can we get rid of force_local (above), remove this for loop, and just rely on param_tag logic instead?
            for p in itertools.chain(context.params, param_ns.params):
                pkey = p.key
                if is_private(pkey):
                    # strip leading ~, which is optional/inferred
                    pkey = pkey[1:]
                pkey = param_ns.ns + pkey
                ros_config.add_param(Param(pkey, p.value), verbose=verbose)
                    
            if not is_test:
                return Node(pkg, node_type, name=name, namespace=child_ns.ns, machine_name=machine, 
                            args=args, respawn=respawn,
                            respawn_delay=respawn_delay,
                            remap_args=remap_context.remap_args(), env_args=env_context.env_args,
                            output=output, cwd=cwd, launch_prefix=launch_prefix,
                            required=required, filename=context.filename)
            else:
                return Test(test_name, pkg, node_type, name=name, namespace=child_ns.ns, 
                            machine_name=machine, args=args,
                            remap_args=remap_context.remap_args(), env_args=env_context.env_args,
                            time_limit=time_limit, cwd=cwd, launch_prefix=launch_prefix,
                            retry=retry, filename=context.filename)
        except KeyError as e:
            raise XmlParseException(
                "<%s> tag is missing required attribute: %s. Node xml is %s"%(tag.tagName, e, tag.toxml()))
        except XmlParseException as e:
            raise XmlParseException(
                "Invalid <node> tag: %s. \n\nNode xml is %s"%(e, tag.toxml()))
        except ValueError as e:
            raise XmlParseException(
                "Invalid <node> tag: %s. \n\nNode xml is %s"%(e, tag.toxml()))

    MACHINE_ATTRS = ('name', 'address', 'env-loader', 
                     'ssh-port', 'user', 'password', 'default', 'timeout')
    @ifunless
    def _machine_tag(self, tag, context, ros_config, verbose=True):
        try:
            # clone context as <machine> tag sets up its own env args
            context = context.child(None)
            
            # pre-fuerte warning attributes
            attrs = self.opt_attrs(tag, context,
                                   ('ros-root', 'ros-package-path', 'ros-ip', 'ros-hostname'))
            if any(attrs):
                raise XmlParseException("<machine>: ros-* attributes are not supported since ROS Fuerte.\nPlease use env-loader instead")

            self._check_attrs(tag, context, ros_config, XmlLoader.MACHINE_ATTRS)
            # required attributes
            name, address = self.reqd_attrs(tag, context, ('name', 'address'))
            
            # optional attributes
            attrs = self.opt_attrs(tag, context,
                                   ('env-loader', 
                                    'ssh-port', 'user', 'password', 'default', 'timeout'))
            env_loader, ssh_port, user, password, default, timeout = attrs

            ssh_port = int(ssh_port or '22')

            # check for default switch
            default = (default or 'false').lower()
            try:
                assignable = _assignable[default]
                is_default = _is_default[default]
            except KeyError as e:
                raise XmlParseException("Invalid value for 'attribute': %s"%default)

            # load env args
            for t in [c for c in tag.childNodes if c.nodeType == DomNode.ELEMENT_NODE]:
                if t.tagName == 'env':
                    raise XmlParseException("<machine>: <env> tag is not supported since ROS Fuerte.\nPlease use env-loader instead")
                else:
                    ros_config.add_config_error("unrecognized '%s' tag in <%s> tag"%(t.tagName, tag.tagName))
            # cast timeout to float. make sure timeout wasn't an empty string or negative
            if timeout:
                try:
                    timeout = float(timeout)
                except ValueError:
                    raise XmlParseException("'timeout' be a number: [%s]"%timeout)
            elif timeout == '':
                raise XmlParseException("'timeout' cannot be empty")
            if timeout is not None and timeout <= 0.:
                raise XmlParseException("'timeout' be a positive number: [%s]"%timeout)                    

            m = Machine(name, address, env_loader=env_loader,
                        ssh_port=ssh_port, user=user, password=password, 
                        assignable=assignable, env_args=context.env_args, timeout=timeout)
            return (m, is_default)
        except KeyError as e:
            raise XmlParseException("<machine> tag is missing required attribute: %s"%e)
        except SubstitutionException as e:
            raise XmlParseException(
                "%s. \n\nMachine xml is %s"%(e, tag.toxml()))
        except RLException as e:
            raise XmlParseException(
                "%s. \n\nMachine xml is %s"%(e, tag.toxml()))
        
    REMAP_ATTRS = ('from', 'to')
    @ifunless
    def _remap_tag(self, tag, context, ros_config):
        try:
            self._check_attrs(tag, context, ros_config, XmlLoader.REMAP_ATTRS)
            return self.reqd_attrs(tag, context, XmlLoader.REMAP_ATTRS)
        except KeyError as e:
            raise XmlParseException("<remap> tag is missing required from/to attributes: %s"%tag.toxml())
        
    ENV_ATTRS = ('name', 'value')
    @ifunless
    def _env_tag(self, tag, context, ros_config):
        try:
            self._check_attrs(tag, context, ros_config, XmlLoader.ENV_ATTRS)
            self.load_env(context, ros_config, *self.reqd_attrs(tag, context, XmlLoader.ENV_ATTRS))
        except ValueError as e:
            raise XmlParseException("Invalid <env> tag: %s. \nXML is %s"%(str(e), tag.toxml()))
        except KeyError as e:
            raise XmlParseException("<env> tag is missing required name/value attributes: %s"%tag.toxml())
    
    def _ns_clear_params_attr(self, tag_name, tag, context, ros_config, node_name=None, include_filename=None):
        """
        Common processing routine for xml tags with NS and CLEAR_PARAMS attributes
        
        @param tag: DOM Node
        @type  tag: Node
        @param context: current namespace context 
        @type  context: LoaderContext
        @param clear_params: list of params to clear
        @type  clear_params: [str]
        @param node_name: name of node (for use when tag_name == 'node')
        @type  node_name: str
        @param include_filename: <include> filename if this is an <include> tag. If specified, context will use include rules.
        @type  include_filename: str
        @return: loader context 
        @rtype:  L{LoaderContext}
        """
        if tag.hasAttribute(NS):
            ns = self.resolve_args(tag.getAttribute(NS), context)
            if not ns:
                raise XmlParseException("<%s> tag has an empty '%s' attribute"%(tag_name, NS))
        else:
            ns = None
        if include_filename is not None:
            child_ns = context.include_child(ns, include_filename)
        else:
            child_ns = context.child(ns)
        clear_p = self.resolve_args(tag.getAttribute(CLEAR_PARAMS), context)
        if clear_p:
            clear_p = _bool_attr(clear_p, False, 'clear_params')
            if clear_p:
                if tag_name == 'node':
                    if not node_name:
                        raise XmlParseException("<%s> tag must have a 'name' attribute to use '%s' attribute"%(tag_name, CLEAR_PARAMS))
                    # use make_global_ns to give trailing slash in order to be consistent with XmlContext.ns
                    ros_config.add_clear_param(make_global_ns(ns_join(child_ns.ns, node_name)))
                else:
                    if not ns:
                        raise XmlParseException("'ns' attribute must be set in order to use 'clear_params'")                
                    ros_config.add_clear_param(child_ns.ns)
        return child_ns
        
    @ifunless
    def _launch_tag(self, tag, ros_config, filename=None):
        # #2499
        deprecated = tag.getAttribute('deprecated')
        if deprecated:
            if filename:
                ros_config.add_config_error("[%s] DEPRECATED: %s"%(filename, deprecated))
            else:
                ros_config.add_config_error("Deprecation Warning: "+deprecated)

    INCLUDE_ATTRS = ('file', NS, CLEAR_PARAMS, 'pass_all_args')
    @ifunless
    def _include_tag(self, tag, context, ros_config, default_machine, is_core, verbose):
        self._check_attrs(tag, context, ros_config, XmlLoader.INCLUDE_ATTRS)
        inc_filename = self.resolve_args(tag.attributes['file'].value, context)

        if tag.hasAttribute('pass_all_args'):
            pass_all_args = self.resolve_args(tag.attributes['pass_all_args'].value, context)
            pass_all_args = _bool_attr(pass_all_args, False, 'pass_all_args')
        else:
            pass_all_args = False

        child_ns = self._ns_clear_params_attr(tag.tagName, tag, context, ros_config, include_filename=inc_filename)

        # If we're asked to pass all args, then we need to add them into the
        # child context.
        if pass_all_args:
            if 'arg' in context.resolve_dict:
                for name, value in context.resolve_dict['arg'].items():
                    child_ns.add_arg(name, value=value)
            # Also set the flag that tells the child context to ignore (rather than
            # error on) attempts to set the same arg twice.
            child_ns.pass_all_args = True

        for t in [c for c in tag.childNodes if c.nodeType == DomNode.ELEMENT_NODE]:
            tag_name = t.tagName.lower()
            if tag_name == 'env':
                self._env_tag(t, child_ns, ros_config)
            elif tag_name == 'arg':
                self._arg_tag(t, child_ns, ros_config, verbose=verbose)
            else:
                print("WARN: unrecognized '%s' tag in <%s> tag"%(t.tagName, tag.tagName), file=sys.stderr)

        # setup arg passing
        loader.process_include_args(child_ns)
                
        try:
            launch = self._parse_launch(inc_filename, verbose=verbose)
            ros_config.add_roslaunch_file(inc_filename)
            self._launch_tag(launch, ros_config, filename=inc_filename)
            default_machine = \
                self._recurse_load(ros_config, launch.childNodes, child_ns, \
                                       default_machine, is_core, verbose)

            # check for unused args
            loader.post_process_include_args(child_ns)

        except ArgException as e:
            raise XmlParseException("included file [%s] requires the '%s' arg to be set"%(inc_filename, str(e)))
        except XmlParseException as e:
            raise XmlParseException("while processing %s:\n%s"%(inc_filename, str(e)))
        if verbose:
            print("... done importing include file [%s]"%inc_filename)
        return default_machine
                
    GROUP_ATTRS = (NS, CLEAR_PARAMS)
    def _recurse_load(self, ros_config, tags, context, default_machine, is_core, verbose):
        """
        @return: new default machine for current context
        @rtype: L{Machine}
        """
        for tag in [t for t in tags if t.nodeType == DomNode.ELEMENT_NODE]:
            name = tag.tagName
            if name == 'group':
                if ifunless_test(self, tag, context):
                    self._check_attrs(tag, context, ros_config, XmlLoader.GROUP_ATTRS)
                    child_ns = self._ns_clear_params_attr(name, tag, context, ros_config)
                    default_machine = \
                        self._recurse_load(ros_config, tag.childNodes, child_ns, \
                                               default_machine, is_core, verbose)
            elif name == 'node':
                n = self._node_tag(tag, context, ros_config, default_machine, verbose=verbose)
                if n is not None:
                    ros_config.add_node(n, core=is_core, verbose=verbose)
            elif name == 'test':
                t = self._node_tag(tag, context, ros_config, default_machine, is_test=True, verbose=verbose)
                if t is not None:
                    ros_config.add_test(t, verbose=verbose)
            elif name == 'param':
                self._param_tag(tag, context, ros_config, verbose=verbose)
            elif name == 'remap':
                try:
                    r = self._remap_tag(tag, context, ros_config)
                    if r is not None:
                        context.add_remap(r)
                except RLException as e:
                    raise XmlParseException("Invalid <remap> tag: %s.\nXML is %s"%(str(e), tag.toxml()))
            elif name == 'machine':
                val = self._machine_tag(tag, context, ros_config, verbose=verbose)
                if val is not None:
                    (m, is_default) = val
                    if is_default:
                        default_machine = m
                    ros_config.add_machine(m, verbose=verbose)
            elif name == 'rosparam':
                self._rosparam_tag(tag, context, ros_config, verbose=verbose)
            elif name == 'master':
                pass #handled non-recursively
            elif name == 'include':
                val = self._include_tag(tag, context, ros_config, default_machine, is_core, verbose)
                if val is not None:
                    default_machine = val
            elif name == 'env':
                self._env_tag(tag, context, ros_config)
            elif name == 'arg':
                self._arg_tag(tag, context, ros_config, verbose=verbose)
            else:
                ros_config.add_config_error("unrecognized tag "+tag.tagName)
        return default_machine

    def _load_launch(self, launch, ros_config, is_core=False, filename=None, argv=None, verbose=True):
        """
        subroutine of launch for loading XML DOM into config. Load_launch assumes that it is
        creating the root XmlContext, and is thus affected by command-line arguments.
        @param launch: DOM node of the root <launch> tag in the file
        @type  launch: L{Node}
        @param ros_config: launch configuration to load XML file into
        @type  ros_config: L{ROSLaunchConfig}
        @param is_core: (optional) if True, load file using ROS core rules. Default False.
        @type  is_core: bool
        @param filename: (optional) name of file being loaded
        @type  filename: str
        @param verbose: (optional) print verbose output. Default False.
        @type  verbose: bool
        @param argv: (optional) command-line args. Default sys.argv.
        """        
        if argv is None:
            argv = sys.argv

        self._launch_tag(launch, ros_config, filename)
        self.root_context = loader.LoaderContext(get_ros_namespace(), filename)
        loader.load_sysargs_into_context(self.root_context, argv)

        if len(launch.getElementsByTagName('master')) > 0:
            print("WARNING: ignoring defunct <master /> tag", file=sys.stderr)
        self._recurse_load(ros_config, launch.childNodes, self.root_context, None, is_core, verbose)
        
    def _parse_launch(self, filename, verbose):
        try:
            if verbose:            
                print("... loading XML file [%s]"%filename)
            root = parse(filename).getElementsByTagName('launch')
        except Exception as e:
            raise XmlParseException("Invalid roslaunch XML syntax: %s"%e)
        if len(root) != 1:
            raise XmlParseException("Invalid roslaunch XML syntax: no root <launch> tag")
        return root[0]
        
    def load(self, filename, ros_config, core=False, argv=None, verbose=True):
        """
        load XML file into launch configuration
        @param filename: XML config file to load
        @type  filename: str
        @param ros_config: launch configuration to load XML file into
        @type  ros_config: L{ROSLaunchConfig}
        @param core: if True, load file using ROS core rules
        @type  core: bool
        @param argv: override command-line arguments (mainly for arg testing)
        @type  argv: [str]
        """
        try:
            launch = self._parse_launch(filename, verbose)
            ros_config.add_roslaunch_file(filename)            
            self._load_launch(launch, ros_config, is_core=core, filename=filename, argv=argv, verbose=verbose)
        except ArgException as e:
            raise XmlParseException("[%s] requires the '%s' arg to be set"%(filename, str(e)))
        except SubstitutionException as e:
            raise XmlParseException(str(e))

    def load_string(self, xml_text, ros_config, core=False, verbose=True):
        """
        Load XML text into launch configuration
        @param xml_text: XML configuration
        @type  xml_text: str
        @param ros_config: launch configuration to load XML file into
        @type  ros_config: L{ROSLaunchConfig}
        @param core: if True, load file using ROS core rules
        @type  core: bool
        """
        try:
            if verbose:
                print("... loading XML")
            try:
                if hasattr(xml_text,'encode') and isinstance(xml_text, unicode):
                    # #3799: xml_text comes in a unicode object, which
                    # #fails since XML text is expected to be encoded.
                    # that's why force encoding to utf-8 here (make sure XML header is utf-8)
                    xml_text = xml_text.encode('utf-8')
            except NameError:
                pass
            root = parseString(xml_text).getElementsByTagName('launch')
        except Exception as e:
            logging.getLogger('roslaunch').error("Invalid roslaunch XML syntax:\nstring[%s]\ntraceback[%s]"%(xml_text, traceback.format_exc()))
            raise XmlParseException("Invalid roslaunch XML syntax: %s"%e)
        
        if len(root) != 1:
            raise XmlParseException("Invalid roslaunch XML syntax: no root <launch> tag")
        self._load_launch(root[0], ros_config, core, filename='string', verbose=verbose)
