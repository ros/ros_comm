#!/usr/bin/env python
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
#

## ROS message source code generation for C++
## 
## Converts ROS .msg files in a package into C++ source code implementations.

import sys
import os
import traceback

import roslib.msgs 
import roslib.packages
import roslib.gentools
from rospkg import RosPack

try:
    from cStringIO import StringIO #Python 2.x
except ImportError:
    from io import StringIO #Python 3.x

MSG_TYPE_TO_CPP = {'byte': 'int8_t', 'char': 'uint8_t',
                   'bool': 'uint8_t',
                   'uint8': 'uint8_t', 'int8': 'int8_t', 
                   'uint16': 'uint16_t', 'int16': 'int16_t', 
                   'uint32': 'uint32_t', 'int32': 'int32_t',
                   'uint64': 'uint64_t', 'int64': 'int64_t',
                   'float32': 'float',
                   'float64': 'double',
                   'string': 'std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > ',
                   'time': 'ros::Time',
                   'duration': 'ros::Duration'}

def msg_type_to_cpp(type):
    """
    Converts a message type (e.g. uint32, std_msgs/String, etc.) into the C++ declaration
    for that type (e.g. uint32_t, std_msgs::String_<ContainerAllocator>)
    
    @param type: The message type
    @type type: str
    @return: The C++ declaration
    @rtype: str
    """
    (base_type, is_array, array_len) = roslib.msgs.parse_type(type)
    cpp_type = None
    if (roslib.msgs.is_builtin(base_type)):
        cpp_type = MSG_TYPE_TO_CPP[base_type]
    elif (len(base_type.split('/')) == 1):
        if (roslib.msgs.is_header_type(base_type)):
            cpp_type = ' ::std_msgs::Header_<ContainerAllocator> '
        else:
            cpp_type = '%s_<ContainerAllocator> '%(base_type)
    else:
        pkg = base_type.split('/')[0]
        msg = base_type.split('/')[1]
        cpp_type = ' ::%s::%s_<ContainerAllocator> '%(pkg, msg)
        
    if (is_array):
        if (array_len is None):
            return 'std::vector<%s, typename ContainerAllocator::template rebind<%s>::other > '%(cpp_type, cpp_type)
        else:
            return 'boost::array<%s, %s> '%(cpp_type, array_len)
    else:
        return cpp_type
    
def cpp_message_declarations(name_prefix, msg):
    """
    Returns the different possible C++ declarations for a message given the message itself.
    
    @param name_prefix: The C++ prefix to be prepended to the name, e.g. "std_msgs::"
    @type name_prefix: str
    @param msg: The message type
    @type msg: str
    @return: A tuple of 3 different names.  cpp_message_decelarations("std_msgs::", "String") returns the tuple
        ("std_msgs::String_", "std_msgs::String_<ContainerAllocator>", "std_msgs::String")
    @rtype: str 
    """
    pkg, basetype = roslib.names.package_resource_name(msg)
    cpp_name = ' ::%s%s'%(name_prefix, msg)
    if (pkg):
        cpp_name = ' ::%s::%s'%(pkg, basetype)
    return ('%s_'%(cpp_name), '%s_<ContainerAllocator> '%(cpp_name), '%s'%(cpp_name))

def write_begin(s, spec, file):
    """
    Writes the beginning of the header file: a comment saying it's auto-generated and the include guards
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The spec
    @type spec: roslib.msgs.MsgSpec
    @param file: The file this message is being generated for
    @type file: str
    """
    s.write("/* Auto-generated by genmsg_cpp for file %s */\n"%(file))
    s.write('#ifndef %s_MESSAGE_%s_H\n'%(spec.package.upper(), spec.short_name.upper()))
    s.write('#define %s_MESSAGE_%s_H\n'%(spec.package.upper(), spec.short_name.upper()))
    
def write_end(s, spec):
    """
    Writes the end of the header file: the ending of the include guards
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The spec
    @type spec: roslib.msgs.MsgSpec
    """
    s.write('#endif // %s_MESSAGE_%s_H\n'%(spec.package.upper(), spec.short_name.upper()))
    
def write_generic_includes(s):
    """
    Writes the includes that all messages need
    
    @param s: The stream to write to
    @type s: stream
    """
    s.write('#include <string>\n')
    s.write('#include <vector>\n')
    s.write('#include <map>\n')
    s.write('#include <ostream>\n')
    s.write('#include "ros/serialization.h"\n')
    s.write('#include "ros/builtin_message_traits.h"\n')
    s.write('#include "ros/message_operations.h"\n')
    s.write('#include "ros/time.h"\n\n')
    s.write('#include "ros/macros.h"\n\n')
    s.write('#include "ros/assert.h"\n\n')
    
def write_includes(s, spec):
    """
    Writes the message-specific includes
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec to iterate over
    @type spec: roslib.msgs.MsgSpec
    """
    for field in spec.parsed_fields():
        if (not field.is_builtin):
            if (field.is_header):
                s.write('#include "std_msgs/Header.h"\n')
            else:
                (pkg, name) = roslib.names.package_resource_name(field.base_type)
                pkg = pkg or spec.package # convert '' to package
                s.write('#include "%s/%s.h"\n'%(pkg, name))
                
    s.write('\n') 
    
    
def write_struct(s, spec, cpp_name_prefix, extra_deprecated_traits = {}):
    """
    Writes the entire message struct: declaration, constructors, members, constants and (deprecated) member functions
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param cpp_name_prefix: The C++ prefix to use when referring to the message, e.g. "std_msgs::"
    @type cpp_name_prefix: str
    """
    
    msg = spec.short_name
    s.write('template <class ContainerAllocator>\n')
    s.write('struct %s_ {\n'%(msg))
    s.write('  typedef %s_<ContainerAllocator> Type;\n\n'%(msg))
    
    write_constructors(s, spec, cpp_name_prefix)
    write_members(s, spec)
    write_constant_declarations(s, spec)
    
    #rospack = RosPack()
    #gendeps_dict = roslib.gentools.get_dependencies(spec, spec.package, compute_files=False, rospack=rospack)
    #md5sum = roslib.gentools.compute_md5(gendeps_dict, rospack=rospack)
    #full_text = compute_full_text_escaped(gendeps_dict)
    
    # write_deprecated_member_functions(s, spec, dict(list({'MD5Sum': md5sum, 'DataType': '%s/%s'%(spec.package, spec.short_name), 'MessageDefinition': full_text}.items()) + list(extra_deprecated_traits.items())))
    
    (cpp_msg_unqualified, cpp_msg_with_alloc, cpp_msg_base) = cpp_message_declarations(cpp_name_prefix, msg)
    s.write('  typedef boost::shared_ptr<%s> Ptr;\n'%(cpp_msg_with_alloc))
    s.write('  typedef boost::shared_ptr<%s const> ConstPtr;\n'%(cpp_msg_with_alloc))

    s.write('}; // struct %s\n'%(msg))
    
    s.write('typedef %s_<std::allocator<void> > %s;\n\n'%(cpp_msg_base, msg))
    s.write('typedef boost::shared_ptr<%s> %sPtr;\n'%(cpp_msg_base, msg))
    s.write('typedef boost::shared_ptr<%s const> %sConstPtr;\n\n'%(cpp_msg_base, msg))

def default_value(type):
    """
    Returns the value to initialize a message member with.  0 for integer types, 0.0 for floating point, false for bool,
    empty string for everything else
    
    @param type: The type
    @type type: str
    """
    if type in ['byte', 'int8', 'int16', 'int32', 'int64',
                'char', 'uint8', 'uint16', 'uint32', 'uint64']:
        return '0'
    elif type in ['float32', 'float64']:
        return '0.0'
    elif type == 'bool':
        return 'false'
        
    return ""

def takes_allocator(type):
    """
    Returns whether or not a type can take an allocator in its constructor.  False for all builtin types except string.
    True for all others.
    
    @param type: The type
    @type: str
    """
    return not type in ['byte', 'int8', 'int16', 'int32', 'int64',
                        'char', 'uint8', 'uint16', 'uint32', 'uint64',
                        'float32', 'float64', 'bool', 'time', 'duration']

def write_initializer_list(s, spec, container_gets_allocator):
    """
    Writes the initializer list for a constructor
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param container_gets_allocator: Whether or not a container type (whether it's another message, a vector, array or string)
        should have the allocator passed to its constructor.  Assumes the allocator is named _alloc.
    @type container_gets_allocator: bool
    """
    
    i = 0
    for field in spec.parsed_fields():
        if (i == 0):
            s.write('  : ')
        else:
            s.write('  , ')
            
        val = default_value(field.base_type)
        use_alloc = takes_allocator(field.base_type)
        if (field.is_array):
            if (field.array_len is None and container_gets_allocator):
                s.write('%s(_alloc)\n'%(field.name))
            else:
                s.write('%s()\n'%(field.name))
        else:
            if (container_gets_allocator and use_alloc):
                s.write('%s(_alloc)\n'%(field.name))
            else:
                s.write('%s(%s)\n'%(field.name, val))
        i = i + 1
        
def write_fixed_length_assigns(s, spec, container_gets_allocator, cpp_name_prefix):
    """
    Initialize any fixed-length arrays
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param container_gets_allocator: Whether or not a container type (whether it's another message, a vector, array or string)
        should have the allocator passed to its constructor.  Assumes the allocator is named _alloc.
    @type container_gets_allocator: bool
    @param cpp_name_prefix: The C++ prefix to use when referring to the message, e.g. "std_msgs::"
    @type cpp_name_prefix: str
    """
    # Assign all fixed-length arrays their default values
    for field in spec.parsed_fields():
        if (not field.is_array or field.array_len is None):
            continue
        
        val = default_value(field.base_type)
        if (container_gets_allocator and takes_allocator(field.base_type)):
            # String is a special case, as it is the only builtin type that takes an allocator
            if (field.base_type == "string"):
                string_cpp = msg_type_to_cpp("string")
                s.write('    %s.assign(%s(_alloc));\n'%(field.name, string_cpp))
            else:
                (cpp_msg_unqualified, cpp_msg_with_alloc, _) = cpp_message_declarations(cpp_name_prefix, field.base_type)
                s.write('    %s.assign(%s(_alloc));\n'%(field.name, cpp_msg_with_alloc))
        elif (len(val) > 0):
            s.write('    %s.assign(%s);\n'%(field.name, val))

def write_constructors(s, spec, cpp_name_prefix):
    """
    Writes any necessary constructors for the message
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param cpp_name_prefix: The C++ prefix to use when referring to the message, e.g. "std_msgs::"
    @type cpp_name_prefix: str
    """
    
    msg = spec.short_name
    
    # Default constructor
    s.write('  %s_()\n'%(msg))
    write_initializer_list(s, spec, False)
    s.write('  {\n')
    write_fixed_length_assigns(s, spec, False, cpp_name_prefix)
    s.write('  }\n\n')
    
    # Constructor that takes an allocator constructor
    s.write('  %s_(const ContainerAllocator& _alloc)\n'%(msg))
    write_initializer_list(s, spec, True)
    s.write('  {\n')
    write_fixed_length_assigns(s, spec, True, cpp_name_prefix)
    s.write('  }\n\n')

def write_member(s, field):
    """
    Writes a single member's declaration and type typedef
    
    @param s: The stream to write to
    @type s: stream
    @param type: The member type
    @type type: str
    @param name: The name of the member
    @type name: str
    """
    cpp_type = msg_type_to_cpp(field.type)
    s.write('  typedef %s _%s_type;\n'%(cpp_type, field.name))
    s.write('  %s %s;\n\n'%(cpp_type, field.name))

def write_members(s, spec):
    """
    Write all the member declarations
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    """
    [write_member(s, field) for field in spec.parsed_fields()]
        
def escape_string(str):
    str = str.replace('\\', '\\\\')
    str = str.replace('"', '\\"')
    return str
        
def write_constant_declaration(s, constant):
    """
    Write a constant value as a static member
    
    @param s: The stream to write to
    @type s: stream
    @param constant: The constant
    @type constant: roslib.msgs.Constant
    """
    
    # integral types get their declarations as enums to allow use at compile time
    if (constant.type in ['byte', 'int8', 'int16', 'int32', 'int64', 'char', 'uint8', 'uint16', 'uint32', 'uint64']):
        s.write('  enum { %s = %s };\n'%(constant.name, constant.val))
    else:
        s.write('  static const %s %s;\n'%(msg_type_to_cpp(constant.type), constant.name))
        
def write_constant_declarations(s, spec):
    """
    Write all the constants from a spec as static members
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    """
    [write_constant_declaration(s, constant) for constant in spec.constants]
    s.write('\n')
    
def write_constant_definition(s, spec, constant):
    """
    Write a constant value as a static member
    
    @param s: The stream to write to
    @type s: stream
    @param constant: The constant
    @type constant: roslib.msgs.Constant
    """
    
    # integral types do not need a definition, since they've been defined where they are declared
    if (constant.type not in ['byte', 'int8', 'int16', 'int32', 'int64', 'char', 'uint8', 'uint16', 'uint32', 'uint64', 'string']):
        s.write('template<typename ContainerAllocator> const %s %s_<ContainerAllocator>::%s = %s;\n'%(msg_type_to_cpp(constant.type), spec.short_name, constant.name, constant.val))
    elif (constant.type == 'string'):
        s.write('template<typename ContainerAllocator> const %s %s_<ContainerAllocator>::%s = "%s";\n'%(msg_type_to_cpp(constant.type), spec.short_name, constant.name, escape_string(constant.val)))
        
def write_constant_definitions(s, spec):
    """
    Write all the constants from a spec as static members
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    """
    [write_constant_definition(s, spec, constant) for constant in spec.constants]
    s.write('\n')
        
def is_fixed_length(spec):
    """
    Returns whether or not the message is fixed-length
    
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param package: The package of the
    @type package: str
    """
    types = []
    for field in spec.parsed_fields():
        if (field.is_array and field.array_len is None):
            return False
        
        if (field.base_type == 'string'):
            return False
        
        if (not field.is_builtin):
            types.append(field.base_type)
            
    types = set(types)
    for type in types:
        type = roslib.msgs.resolve_type(type, spec.package)
        (_, new_spec) = roslib.msgs.load_by_type(type, spec.package)
        if (not is_fixed_length(new_spec)):
            return False
        
    return True
    
def write_deprecated_member_functions(s, spec, traits):
    """
    Writes the deprecated member functions for backwards compatibility
    """
    for field in spec.parsed_fields():
        if (field.is_array):
            s.write('  ROS_DEPRECATED uint32_t get_%s_size() const { return (uint32_t)%s.size(); }\n'%(field.name, field.name))
            
            if (field.array_len is None):
                s.write('  ROS_DEPRECATED void set_%s_size(uint32_t size) { %s.resize((size_t)size); }\n'%(field.name, field.name))
                s.write('  ROS_DEPRECATED void get_%s_vec(%s& vec) const { vec = this->%s; }\n'%(field.name, msg_type_to_cpp(field.type), field.name))
                s.write('  ROS_DEPRECATED void set_%s_vec(const %s& vec) { this->%s = vec; }\n'%(field.name, msg_type_to_cpp(field.type), field.name))
    
    for k, v in traits.items():
        s.write('private:\n')
        s.write('  static const char* __s_get%s_() { return "%s"; }\n'%(k, v))
        s.write('public:\n')
        s.write('  ROS_DEPRECATED static const std::string __s_get%s() { return __s_get%s_(); }\n\n'%(k, k))
        s.write('  ROS_DEPRECATED const std::string __get%s() const { return __s_get%s_(); }\n\n'%(k, k))
    
    s.write('  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const\n  {\n')
    s.write('    ros::serialization::OStream stream(write_ptr, 1000000000);\n')
    for field in spec.parsed_fields():
        s.write('    ros::serialization::serialize(stream, %s);\n'%(field.name))
    s.write('    return stream.getData();\n  }\n\n')
    
    s.write('  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)\n  {\n')
    s.write('    ros::serialization::IStream stream(read_ptr, 1000000000);\n');
    for field in spec.parsed_fields():
        s.write('    ros::serialization::deserialize(stream, %s);\n'%(field.name))
    s.write('    return stream.getData();\n  }\n\n')
    
    s.write('  ROS_DEPRECATED virtual uint32_t serializationLength() const\n  {\n')
    s.write('    uint32_t size = 0;\n');
    for field in spec.parsed_fields():
        s.write('    size += ros::serialization::serializationLength(%s);\n'%(field.name))
    s.write('    return size;\n  }\n\n')

def compute_full_text_escaped(gen_deps_dict):
    """
    Same as roslib.gentools.compute_full_text, except that the
    resulting text is escaped to be safe for C++ double quotes

    @param get_deps_dict: dictionary returned by get_dependencies call
    @type  get_deps_dict: dict
    @return: concatenated text for msg/srv file and embedded msg/srv types. Text will be escaped for double quotes
    @rtype: str
    """
    definition = roslib.gentools.compute_full_text(gen_deps_dict)
    lines = definition.split('\n')
    s = StringIO()
    for line in lines:
        line = escape_string(line)
        s.write('%s\\n\\\n'%(line))
        
    val = s.getvalue()
    s.close()
    return val

def is_hex_string(str):
    for c in str:
        if c not in '0123456789abcdefABCDEF':
            return False
        
    return True

def write_trait_char_class(s, class_name, cpp_msg_with_alloc, value, write_static_hex_value = False):
    """
    Writes a class trait for traits which have static value() members that return const char*
    
    e.g. write_trait_char_class(s, "MD5Sum", "std_msgs::String_<ContainerAllocator>", "hello") yields:
    template<class ContainerAllocator>
    struct MD5Sum<std_msgs::String_<ContainerAllocator> > 
    {
        static const char* value() { return "hello"; }
        static const char* value(const std_msgs::String_<ContainerAllocator>&) { return value(); }
    };
    
    @param s: The stream to write to
    @type s: stream
    @param class_name: The name of the trait class to write
    @type class_name: str
    @param cpp_msg_with_alloc: The C++ declaration of the message, including the allocator template
    @type cpp_msg_with_alloc: str
    @param value: The value to return in the string
    @type value: str
    @param write_static_hex_value: Whether or not to write a set of compile-time-checkable static values.  Useful for,
        for example, MD5Sum.  Writes static const uint64_t static_value1... static_valueN
    @type write_static_hex_value: bool
    @raise ValueError if write_static_hex_value is True but value contains characters invalid in a hex value
    """
    s.write('template<class ContainerAllocator>\nstruct %s<%s> {\n'%(class_name, cpp_msg_with_alloc))
    s.write('  static const char* value() \n  {\n    return "%s";\n  }\n\n'%(value))
    s.write('  static const char* value(const %s&) { return value(); } \n'%(cpp_msg_with_alloc))
    if (write_static_hex_value):
        if (not is_hex_string(value)):
            raise ValueError('%s is not a hex value'%(value))
        
        iter_count = len(value) / 16
        for i in range(0, int(iter_count)):
            start = i*16
            s.write('  static const uint64_t static_value%s = 0x%sULL;\n'%((i+1), value[start:start+16]))
    s.write('};\n\n')
    
def write_trait_true_class(s, class_name, cpp_msg_with_alloc):
    """
    Writes a true/false trait class
    
    @param s: stream to write to
    @type s: stream
    @param class_name: Name of the trait class
    @type class_name: str
    @param cpp_msg_with_alloc: The C++ declaration of the message, including the allocator template
    @type cpp_msg_with_alloc: str
    """
    s.write('template<class ContainerAllocator> struct %s<%s> : public TrueType {};\n'%(class_name, cpp_msg_with_alloc))

def write_traits(s, spec, cpp_name_prefix, datatype = None, rospack=None):
    """
    Writes all the traits classes for a message
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param cpp_name_prefix: The C++ prefix to prepend to a message to refer to it (e.g. "std_msgs::")
    @type cpp_name_prefix: str
    @param datatype: The string to write as the datatype of the message.  If None (default), pkg/msg is used.
    @type datatype: str
    """
    # generate dependencies dictionary
    gendeps_dict = roslib.gentools.get_dependencies(spec, spec.package, compute_files=False, rospack=rospack)
    md5sum = roslib.gentools.compute_md5(gendeps_dict, rospack=rospack)
    full_text = compute_full_text_escaped(gendeps_dict)
    
    if (datatype is None):
        datatype = '%s'%(spec.full_name)
    
    (cpp_msg_unqualified, cpp_msg_with_alloc, _) = cpp_message_declarations(cpp_name_prefix, spec.short_name)
    s.write('namespace ros\n{\n')
    s.write('namespace message_traits\n{\n')

    write_trait_true_class(s, 'IsMessage', cpp_msg_with_alloc)
    write_trait_true_class(s, 'IsMessage', cpp_msg_with_alloc + " const")

    write_trait_char_class(s, 'MD5Sum', cpp_msg_with_alloc, md5sum, True)
    write_trait_char_class(s, 'DataType', cpp_msg_with_alloc, datatype)
    write_trait_char_class(s, 'Definition', cpp_msg_with_alloc, full_text)
    
    if (spec.has_header()):
        write_trait_true_class(s, 'HasHeader', cpp_msg_with_alloc)
        write_trait_true_class(s, 'HasHeader', ' const' + cpp_msg_with_alloc)

    if (is_fixed_length(spec)):
        write_trait_true_class(s, 'IsFixedSize', cpp_msg_with_alloc)
        
    s.write('} // namespace message_traits\n')
    s.write('} // namespace ros\n\n')
    
def write_operations(s, spec, cpp_name_prefix):
    (cpp_msg_unqualified, cpp_msg_with_alloc, _) = cpp_message_declarations(cpp_name_prefix, spec.short_name)
    s.write('namespace ros\n{\n')
    s.write('namespace message_operations\n{\n')
    
    # Write the Printer operation
    s.write('\ntemplate<class ContainerAllocator>\nstruct Printer<%s>\n{\n'%(cpp_msg_with_alloc))
    s.write('  template<typename Stream> static void stream(Stream& s, const std::string& indent, const %s& v) \n  {\n'%cpp_msg_with_alloc)
    for field in spec.parsed_fields():
        cpp_type = msg_type_to_cpp(field.base_type)
        if (field.is_array):
            s.write('    s << indent << "%s[]" << std::endl;\n'%(field.name))
            s.write('    for (size_t i = 0; i < v.%s.size(); ++i)\n    {\n'%(field.name))
            s.write('      s << indent << "  %s[" << i << "]: ";\n'%field.name)
            indent_increment = '  '
            if (not field.is_builtin):
                s.write('      s << std::endl;\n')
                s.write('      s << indent;\n')
                indent_increment = '    ';
            s.write('      Printer<%s>::stream(s, indent + "%s", v.%s[i]);\n'%(cpp_type, indent_increment, field.name))
            s.write('    }\n')
        else:
            s.write('    s << indent << "%s: ";\n'%field.name)
            indent_increment = '  '
            if (not field.is_builtin or field.is_array):
                s.write('s << std::endl;\n')
            s.write('    Printer<%s>::stream(s, indent + "%s", v.%s);\n'%(cpp_type, indent_increment, field.name))
    s.write('  }\n')
    s.write('};\n\n')
        
    s.write('\n')
        
    s.write('} // namespace message_operations\n')
    s.write('} // namespace ros\n\n')
    
def write_serialization(s, spec, cpp_name_prefix):
    """
    Writes the Serializer class for a message
    
    @param s: Stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param cpp_name_prefix: The C++ prefix to prepend to a message to refer to it (e.g. "std_msgs::")
    @type cpp_name_prefix: str
    """
    (cpp_msg_unqualified, cpp_msg_with_alloc, _) = cpp_message_declarations(cpp_name_prefix, spec.short_name)
    
    s.write('namespace ros\n{\n')
    s.write('namespace serialization\n{\n\n')
    
    s.write('template<class ContainerAllocator> struct Serializer<%s>\n{\n'%(cpp_msg_with_alloc))
    
    s.write('  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)\n  {\n')
    for field in spec.parsed_fields():
        s.write('    stream.next(m.%s);\n'%(field.name))
    s.write('  }\n\n')
    
    s.write('  ROS_DECLARE_ALLINONE_SERIALIZER;\n')
    
    s.write('}; // struct %s_\n'%(spec.short_name))
        
    s.write('} // namespace serialization\n')
    s.write('} // namespace ros\n\n')
    
def write_ostream_operator(s, spec, cpp_name_prefix):
    (cpp_msg_unqualified, cpp_msg_with_alloc, _) = cpp_message_declarations(cpp_name_prefix, spec.short_name)
    s.write('template<typename ContainerAllocator>\nstd::ostream& operator<<(std::ostream& s, const %s& v)\n{\n'%(cpp_msg_with_alloc))
    s.write('  ros::message_operations::Printer<%s>::stream(s, "", v);\n  return s;}\n\n'%(cpp_msg_with_alloc))

def generate(msg_path):
    """
    Generate a message
    
    @param msg_path: The path to the .msg file
    @type msg_path: str
    """
    (package_dir, package) = roslib.packages.get_dir_pkg(msg_path)
    (_, spec) = roslib.msgs.load_from_file(msg_path, package)
    
    s = StringIO()
    write_begin(s, spec, msg_path)
    write_generic_includes(s)
    write_includes(s, spec)
    
    cpp_prefix = '%s::'%(package)
    
    s.write('namespace %s\n{\n'%(package))
    write_struct(s, spec, cpp_prefix)
    write_constant_definitions(s, spec)
    write_ostream_operator(s, spec, cpp_prefix)
    s.write('} // namespace %s\n\n'%(package))
    
    rospack = RosPack()
    write_traits(s, spec, cpp_prefix, rospack=rospack)
    write_serialization(s, spec, cpp_prefix)
    write_operations(s, spec, cpp_prefix)
    
    # HACK HACK HACK.  The moving of roslib/Header causes many problems.  We end up having to make roslib/Header act exactly
    # like std_msgs/Header (as in, constructor that takes it, as well as operator std_msgs::Header()), and it needs to be
    # available wherever std_msgs/Header.h has been included
    if (package == "std_msgs" and spec.short_name == "Header"):
        s.write("#define STD_MSGS_INCLUDING_HEADER_DEPRECATED_DEF 1\n")
        s.write("#include <std_msgs/header_deprecated_def.h>\n")
        s.write("#undef STD_MSGS_INCLUDING_HEADER_DEPRECATED_DEF\n\n") 
    
    write_end(s, spec)
    
    output_dir = '%s/msg_gen/cpp/include/%s'%(package_dir, package)
    if (not os.path.exists(output_dir)):
        # if we're being run concurrently, the above test can report false but os.makedirs can still fail if
        # another copy just created the directory
        try:
            os.makedirs(output_dir)
        except OSError as e:
            pass
         
    f = open('%s/%s.h'%(output_dir, spec.short_name), 'w')
    f.write(s.getvalue() + "\n")
    
    s.close()

def generate_messages(argv):
    for arg in argv[1:]:
        generate(arg)

if __name__ == "__main__":
    roslib.msgs.set_verbose(False)
    generate_messages(sys.argv)

