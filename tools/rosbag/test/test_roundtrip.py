import unittest
import tempfile
import os

import rosbag
import rospy

BAG_DIR = tempfile.mkdtemp(prefix='rosbag_tests')

class TestRoundTrip(unittest.TestCase):
    def _write_simple_bag(self, name):
        from std_msgs.msg import Int32, String

        with rosbag.Bag(name, 'w') as bag:
            s = String(data='foo')
            i = Int32(data=42)

            bag.write('chatter', s)
            bag.write('numbers', i)

    def _fname(self, name):
        return os.path.join(BAG_DIR, name)

    def test_value_equality(self):
        fname = self._fname('test_value_equality.bag')

        self._write_simple_bag(fname)

        with rosbag.Bag(fname) as bag:
            numbers = list(bag.read_messages('numbers'))
            chatter = list(bag.read_messages('chatter'))

        self.assertEqual(len(numbers), 1)
        self.assertEqual(len(chatter), 1)

        numbers = numbers[0]
        chatter = chatter[0]

        # channel names
        self.assertEqual(numbers[0], 'numbers')
        self.assertEqual(chatter[0], 'chatter')

        # values
        self.assertEqual(numbers[1].data, 42)
        self.assertEqual(chatter[1].data, 'foo')

    @unittest.expectedFailure
    def test_type_equality(self):
        fname = self._fname('test_type_equality.bag')

        from std_msgs.msg import Int32, String

        self._write_simple_bag(fname)

        with rosbag.Bag(fname) as bag:
            numbers = next(bag.read_messages('numbers'))
            chatter = next(bag.read_messages('chatter'))

        self.assertEqual(numbers[1].__class__, Int32)
        self.assertEqual(chatter[1].__class__, String)

    @unittest.expectedFailure
    def test_type_isinstance(self):
        fname = self._fname('test_type_isinstance.bag')

        from std_msgs.msg import Int32, String

        self._write_simple_bag(fname)

        with rosbag.Bag(fname) as bag:
            numbers = next(bag.read_messages('numbers'))
            chatter = next(bag.read_messages('chatter'))

        self.assertIsInstance(numbers[1], Int32)
        self.assertIsInstance(chatter[1], String)
