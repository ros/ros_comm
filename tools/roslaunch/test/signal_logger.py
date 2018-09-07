#!/usr/bin/env python

import os
import signal
import tempfile
import time


LOG_FILE = os.path.join(tempfile.gettempdir(), "signal.log")
log_stream = open(LOG_FILE, 'w')


def handler(signum, frame):
    log_stream.write("%i %s\n" % (signum, str(time.time())))
    log_stream.flush()

    if signum == signal.SIGTERM:
        log_stream.close()


signal.signal(signal.SIGINT, handler)
signal.signal(signal.SIGTERM, handler)

while True:
    time.sleep(10)
