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
from os.path import join, exists

from test_xmlloader import get_test_path, RosLaunchMock

import roslaunch.loader 
import roslaunch.xmlloader
from roslaunch.xmlloader import XmlParseException

import unittest


class TestExportAllArgs(unittest.TestCase):
    def setUp(self):
        self.xml_dir = get_test_path()
        
    def _load(self, test_file, argv=None):
        test_file = join(self.xml_dir, test_file)

        loader = roslaunch.xmlloader.XmlLoader()
        mock = RosLaunchMock()
        self.assert_(exists(test_file),
                     "cannot locate test file %s" % test_file)
        loader.load(test_file, mock, argv=argv)
        return loader.root_context, mock
        
    def test_single_level(self):
        args = []
        filename = "test-export-single-level.xml"

        # Try first, without specifying the value of the arg exported
        # from the second level that has no value or default (this will
        # result in an error)
        try:
            context, mock = self._load(filename, argv=args)
            self.fail("Expected an error due to missing arg_with_no_value_or_default")
        except XmlParseException:
            pass  # Expected an error

        # Now try, and specify the value of the missing arg (no error)
        args.append("arg_with_no_value_or_default:=hello")
        context, mock = self._load("test-export-single-level.xml", argv=args)

        # The main launch file contains the following args:
        #     my_arg (default = my_value)
        #
        # The exported launch file contains the following args:
        #     arg_with_no_value_or_default
        #     arg_with_value
        #     arg_with_default (default = default_value)

        # Check that all expected args are defined
        assert "my_arg" in context.arg_names
        assert "arg_with_no_value_or_default" in context.arg_names
        assert "arg_with_default" in context.arg_names

        # The arg with a value set is NOT exported to the parent launch file
        # since it cannot be overriden
        assert "arg_with_value" not in context.arg_names

        # Now verify the values of the args
        assert "arg" in context.resolve_dict
        assert "my_arg" in context.resolve_dict["arg"]
        assert "arg_with_no_value_or_default" in context.resolve_dict["arg"]
        assert "arg_with_default" in context.resolve_dict["arg"]
        assert "my_value" == context.resolve_dict["arg"]["my_arg"]
        assert "hello" == context.resolve_dict["arg"]["arg_with_no_value_or_default"]
        assert "default_value" == context.resolve_dict["arg"]["arg_with_default"]

        # Now, verify doc strings also get exported
        assert "arg_doc" in context.resolve_dict
        assert "my_arg" in context.resolve_dict["arg_doc"]
        assert "arg_with_no_value_or_default" in context.resolve_dict["arg_doc"]
        assert "arg_with_default" in context.resolve_dict["arg_doc"]
        assert None == context.resolve_dict["arg_doc"]["my_arg"][0]
        assert u"my_value" == context.resolve_dict["arg_doc"]["my_arg"][1]
        assert u"my docstring" == context.resolve_dict["arg_doc"]["arg_with_no_value_or_default"][0]
        assert None == context.resolve_dict["arg_doc"]["arg_with_no_value_or_default"][1]
        assert None == context.resolve_dict["arg_doc"]["arg_with_default"][0]
        assert u"default_value" == context.resolve_dict["arg_doc"]["arg_with_default"][1]

        # Test nodes, should be one
        assert len(mock.nodes) == 1

        # Check node params
        node = mock.nodes[0]
        assert "test_single_level" == node.name
        assert len(mock.params) == 3
        assert "/%s/param1=hello" % node.name == str(mock.params[0])
        assert "/%s/param2=fixed_value" % node.name == str(mock.params[1])
        assert "/%s/param3=default_value" % node.name == str(mock.params[2])

    def test_multiple_levels(self):
        args = []
        filename = "test-export-multiple-levels.xml"

        # Try first, without specifying the value of the arg exported
        # from the second level that has no value or default (this will
        # result in an error)
        try:
            context, mock = self._load(filename, argv=args)
            self.fail("Expected an error due to missing child1_arg")
        except XmlParseException, e:
            pass  # Expected an error

        # Now try, and specify the value of the missing arg (still error)
        try:
            args.append("child1_arg:=true")
            context, mock = self._load(filename, argv=args)
            self.fail("Expected an error due to missing child2_arg")
        except XmlParseException, e:
            pass  # Expected an error

        # Specify the other missing argument (no error now)
        args.append("child2_arg:=123")
        context, mock = self._load(filename, argv=args)

        # The main launch file contains the following args:
        #     my_main_arg (default = my_main_value)
        #
        # The child1 launch file contains the following args:
        #     child_1_arg
        #     child1_arg_value
        #     chil1_arg_default (default = a_default_value)
        #
        # The child2 launch file contains the following args:
        #     child2_arg
        #     child2_arg_value
        #     child2_arg_default (default = a_default_value)

        # Check that all expected args are defined
        assert "my_main_arg" in context.arg_names
        assert "child1_arg" in context.arg_names
        assert "child1_arg_default" in context.arg_names
        assert "child2_arg" in context.arg_names
        assert "child2_arg_default" in context.arg_names

        # Fixed args do not get exported
        assert "child1_arg_value" not in context.arg_names
        assert "child2_arg_value" not in context.arg_names

        # Check values of each arg
        assert "arg" in context.resolve_dict
        assert "my_main_arg" in context.resolve_dict["arg"]
        assert "child1_arg" in context.resolve_dict["arg"]
        assert "child1_arg_default" in context.resolve_dict["arg"]
        assert "child2_arg" in context.resolve_dict["arg"]
        assert "child2_arg_default" in context.resolve_dict["arg"]
        assert "my_main_value" in context.resolve_dict["arg"]["my_main_arg"]
        assert "true" in context.resolve_dict["arg"]["child1_arg"]
        assert "a_default_value" in context.resolve_dict["arg"]["child1_arg_default"]
        assert "123" in context.resolve_dict["arg"]["child2_arg"]
        assert "a_default_value" in context.resolve_dict["arg"]["child2_arg_default"]

        # Check doc strings for each arg
        assert "arg_doc" in context.resolve_dict
        assert "my_main_arg" in context.resolve_dict["arg_doc"]
        assert "child1_arg" in context.resolve_dict["arg_doc"]
        assert "child1_arg_default" in context.resolve_dict["arg_doc"]
        assert "child2_arg" in context.resolve_dict["arg_doc"]
        assert "child2_arg_default" in context.resolve_dict["arg_doc"]
        assert None == context.resolve_dict["arg_doc"]["my_main_arg"][0]
        assert "my_main_value" == context.resolve_dict["arg_doc"]["my_main_arg"][1]
        assert None == context.resolve_dict["arg_doc"]["child1_arg"][0]
        assert None == context.resolve_dict["arg_doc"]["child1_arg"][1]
        assert "child1 default docstring" == context.resolve_dict["arg_doc"]["child1_arg_default"][0]
        assert "a_default_value" == context.resolve_dict["arg_doc"]["child1_arg_default"][1]
        assert "child2 arg docstring" == context.resolve_dict["arg_doc"]["child2_arg"][0]
        assert None == context.resolve_dict["arg_doc"]["child2_arg"][1]
        assert "child2 default docstring" == context.resolve_dict["arg_doc"]["child2_arg_default"][0]
        assert "a_default_value" == context.resolve_dict["arg_doc"]["child2_arg_default"][1]

        # Test nodes, should be two
        assert len(mock.nodes) == 2

        # Check node params
        node2, node1 = mock.nodes
        assert "child1_node" == node1.name
        assert "child2_node" == node2.name
        assert len(mock.params) == 6
        assert "/%s/param1=123" % node2.name == str(mock.params[0])
        assert "/%s/param2=a_fixed_value" % node2.name == str(mock.params[1])
        assert "/%s/param3=a_default_value" % node2.name == str(mock.params[2])
        assert "/%s/param1=True" % node1.name == str(mock.params[3])
        assert "/%s/param2=a_fixed_value" % node1.name == str(mock.params[4])
        assert "/%s/param3=a_default_value" % node1.name == str(mock.params[5])

    def test_export_no_args(self):
        args = []
        filename = "test-export-no-args.xml"

        context, mock = self._load(filename, argv=args)

        assert len(context.arg_names) == 0

    def test_top_level_override_export(self):
        args = []
        filename = "test-export-top-level-override-export.xml"

        context, mock = self._load(filename, argv=args)

        # Test that the arg is defined, as the value specified in the
        # top level launch file
        assert "my_arg" in context.arg_names
        assert "arg" in context.resolve_dict
        assert "my_arg" in context.resolve_dict["arg"]
        assert u"top_level_value" == context.resolve_dict["arg"]["my_arg"]

        # Pass a new value for the argument
        args.append("my_arg:=hello")
        context, mock = self._load(filename, argv=args)

        # Test that the new value is set
        assert "my_arg" in context.arg_names
        assert "arg" in context.resolve_dict
        assert "my_arg" in context.resolve_dict["arg"]
        assert u"hello" == context.resolve_dict["arg"]["my_arg"]

        # Test nodes, should be one
        assert len(mock.nodes) == 1

        # Check node params
        node = mock.nodes[0]
        assert "test_top_level_override_export_child" == node.name
        assert len(mock.params) == 1
        assert "/%s/param1=hello" % node.name == str(mock.params[0])

    def test_top_level_pass_exported_arg(self):
        args = []
        filename = "test-export-top-level-pass-exported-arg.xml"

        context, mock = self._load(filename, argv=args)

        # Expected param values
        arg_value = u"this is my value"
        second_value = u"this was set in the top level file"

        # Test that the arg is specified
        assert "custom_arg" in context.arg_names
        assert "child_arg" in context.arg_names
        assert "arg" in context.resolve_dict
        assert "custom_arg" in context.resolve_dict["arg"]
        assert "another_arg" in context.resolve_dict["arg"]
        assert "child_arg" in context.resolve_dict["arg"]
        assert arg_value == context.resolve_dict["arg"]["custom_arg"]

        # The args exported from the child take the value set in the
        # child include file
        assert u"this was set in the child launch file" == context.resolve_dict["arg"]["another_arg"]
        assert u"this is the child value" == context.resolve_dict["arg"]["child_arg"]

        # Test the child include has the node, and the parameter specified
        # has the value that is set in the top level launch file
        assert len(mock.nodes) == 1
        the_node = mock.nodes[0]
        assert "test_pass_exported_arg" == the_node.name

        # The node should have two parameters
        assert len(mock.params) == 2
        assert "/%s/test_param=%s" % (the_node.name, arg_value) == str(mock.params[0])
        assert "/%s/second_param=%s" % (the_node.name, second_value) == str(mock.params[1])

    def test_two_child_exporting_same_arg(self):
        args = []
        filename = "test-export-two-children-exporting-same-arg.xml"

        context, mock = self._load(filename, argv=args)

        child1_arg_value = "child1_default"

        # Check args
        assert "the_arg" in context.arg_names
        assert "arg" in context.resolve_dict
        assert "the_arg" in context.resolve_dict["arg"]
        assert child1_arg_value == str(context.resolve_dict["arg"]["the_arg"])

        # Should be two nodes
        assert len(mock.nodes) == 2

        # Check child 1
        child1_node = mock.nodes[0]
        assert "child1_node" == child1_node.name
        assert "/%s/the_param=%s" % (child1_node.name, child1_arg_value) == str(mock.params[0])

        # Check child 2
        # NOTE: the child2 node param takes on the default value from
        #       the child1 arg because it is defined in the parent
        #       namespace first
        child2_node = mock.nodes[1]
        assert "child2_node" == child2_node.name
        assert "/%s/the_param=%s" % (child2_node.name, child1_arg_value) == str(mock.params[1])

        # Test the same file, but override the arg
        child1_arg_value = "override_value"
        args.append("the_arg:=%s" % child1_arg_value)
        context, mock = self._load(filename, argv=args)

        # Check args
        assert "the_arg" in context.arg_names
        assert "arg" in context.resolve_dict
        assert "the_arg" in context.resolve_dict["arg"]
        assert child1_arg_value == str(context.resolve_dict["arg"]["the_arg"])

        # Should be two nodes
        assert len(mock.nodes) == 2

        # Check child 1
        child1_node = mock.nodes[0]
        assert "child1_node" == child1_node.name
        assert "/%s/the_param=%s" % (child1_node.name, child1_arg_value) == str(mock.params[0])

        # Check child 2
        child2_node = mock.nodes[1]
        assert "child2_node" == child2_node.name
        assert "/%s/the_param=%s" % (child2_node.name, child1_arg_value) == str(mock.params[1])

    def test_export_with_group(self):
        args = []
        filename = "test-export-with-group.xml"

        context, mock = self._load(filename, argv=args)

        arg_def_value = "my_value"

        # Check args
        assert "the_arg" in context.arg_names
        assert "arg" in context.resolve_dict
        assert "the_arg" in context.resolve_dict["arg"]
        assert arg_def_value == str(context.resolve_dict["arg"]["the_arg"])

        # Should be one node
        assert len(mock.nodes) == 1

        # Check child 1
        node = mock.nodes[0]
        assert "test_export_with_group" == node.name
        assert "/my_group/%s/the_param=%s" % (node.name, arg_def_value) == str(mock.params[0])

        # Test overriding the argument
        arg_value = "new arg value"
        args.append("the_arg:=%s" % arg_value)
        context, mock = self._load(filename, argv=args)

        # Check args
        assert "the_arg" in context.arg_names
        assert "arg" in context.resolve_dict
        assert "the_arg" in context.resolve_dict["arg"]
        assert arg_value == str(context.resolve_dict["arg"]["the_arg"])

        # Should be one node
        assert len(mock.nodes) == 1

        # Check child 1
        node = mock.nodes[0]
        assert "test_export_with_group" == node.name
        assert "/my_group/%s/the_param=%s" % (node.name, arg_value) == str(mock.params[0])

