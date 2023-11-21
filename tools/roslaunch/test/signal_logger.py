#!/usr/bin/env python3

import os
import signal
import tempfile
import time

if __name__ == '__main__':
    LOG_FILE = os.path.join(tempfile.gettempdir(), "signal.log")
    log_stream = open(LOG_FILE, 'w')


    def handler(signum, _):
        log_stream.write("%i %s\n" % (signum, str(time.time())))

        if signum == signal.SIGTERM:
            # note: Python's IO stack is not reentrant.
            #       Writing to a file in signal handling is unsafe and raises
            #       RuntimeError on Python 3 when flush() is called during flush().
            #       Call flush() only on SIGTERM to reduce a chance of reentrance.
            #       See https://bugs.python.org/issue24283
            log_stream.flush()
            log_stream.close()


    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    while True:
        time.sleep(10)
