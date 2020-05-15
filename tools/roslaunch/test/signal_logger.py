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
        log_stream.flush()

        if signum == signal.SIGTERM:
            log_stream.close()


    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    while True:
        time.sleep(10)
