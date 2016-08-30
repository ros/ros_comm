import logging
import os
from StringIO import StringIO
import sys

from nose.tools import assert_regexp_matches
import rosgraph.roslogging


os.environ['ROSCONSOLE_FORMAT'] = ' '.join([
    '${severity}',
    '${message}',
    '${walltime}',
    '${thread}',
    '${logger}',
    '${file}',
    '${line}',
    '${function}',
    '${node}',
    '${time}',
])
rosgraph.roslogging.configure_logging('test_rosgraph', logging.INFO)
loginfo = logging.getLogger('rosout').info

# Remap stdout for testing
f = StringIO()
sys.stdout = f


loginfo('on module')


def logging_on_function():
    loginfo('on function')

logging_on_function()


class LoggingOnClass(object):

    def __init__(self):
        loginfo('on method')

LoggingOnClass()


def test_rosconsole__logging_format():
    this_file = os.path.abspath(__file__)
    # this is necessary to avoid test fails because of .pyc cache file
    base, ext = os.path.splitext(this_file)
    if ext == '.pyc':
        this_file = base + '.py'

    for i, loc in enumerate(['module', 'function', 'method']):
        if loc == 'module':
            function = '<module>'
        elif loc == 'function':
            function = 'logging_on_function'
        elif loc == 'method':
            function = 'LoggingOnClass.__init__'
        else:
            raise ValueError

        log_out = ' '.join([
            'INFO',
            'on ' + loc,
            '[0-9]*\.[0-9]*',
            '[0-9]*',
            'rosout',
            this_file,
            '[0-9]*',
            function,
            '/unnamed',
            '[0-9]*\.[0-9]*',
        ])
        assert_regexp_matches(f.getvalue().splitlines()[i], log_out)


sys.stdout = sys.__stdout__
