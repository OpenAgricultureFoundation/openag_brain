#!/usr/bin/env python
import gevent.monkey; gevent.monkey.patch_all()
from gevent.wsgi import WSGIServer

from openag_brain.api import app

if __name__ == '__main__':
    server = WSGIServer(('', 5000), app)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.stop()
