#!/usr/bin/env python

# This file should be run using a non-ros unit test framework such as nose using
# nosetests test_dotname.py.

import unittest
import rostest
from dotname_cases import DotnameLoadingTest, NotTestCase, DotnameLoadingSuite
import exceptions

class TestDotnameLoading(unittest.TestCase):

    def test_class_basic(self):
        rostest.rosrun('test_rostest', 'test_class_basic', DotnameLoadingTest)

    def test_class_dotname(self):
        rostest.rosrun('test_rostest', 'test_class_dotname', 'dotname_cases.DotnameLoadingTest')

    def test_method_dotname(self):
        rostest.rosrun('test_rostest', 'test_method_dotname', 'dotname_cases.DotnameLoadingTest.test_a')

    def test_suite_dotname(self):
        rostest.rosrun('test_rostest', 'test_suite_dotname', 'dotname_cases.DotnameLoadingSuite')

    def test_class_basic_nottest(self):
        # class which exists but is not a TestCase
        with self.assertRaises(SystemExit):
            rostest.rosrun('test_rostest', 'test_class_basic_nottest', NotTestCase)

    def test_suite_basic(self):
        # can't load suites with the basic loader
        with self.assertRaises(SystemExit):
            rostest.rosrun('test_rosunit', 'test_suite_basic', DotnameLoadingSuite)

    def test_class_dotname_nottest(self):
        # class which exists but is not a valid test
        with self.assertRaises(TypeError):
            rostest.rosrun('test_rostest', 'test_class_dotname_nottest', 'dotname_cases.NotTestCase')

    def test_class_dotname_noexist(self):
        # class which does not exist in the module
        with self.assertRaises(AttributeError):
            rostest.rosrun('test_rostest', 'test_class_dotname_noexist', 'dotname_cases.DotnameLoading')

    def test_method_dotname_noexist(self):
        # method which does not exist in the class
        with self.assertRaises(AttributeError):
            rostest.rosrun('test_rostest', 'test_method_dotname_noexist', 'dotname_cases.DotnameLoadingTest.not_method')


if __name__ == '__main__':
    unittest.main()
