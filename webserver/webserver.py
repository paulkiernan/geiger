# Standard Library Imports
from uuid import uuid4

# Third Party Library Imports
import gevent
assert gevent  # Silence gevent unused import errors
from gevent.wsgi import WSGIServer
from gevent.queue import Queue
from gevent.queue import Empty
import serial

# Flask Imports
from flask import Flask
from flask import render_template
from flask import Response

# Global Variables

APP = Flask(__name__)
SERIAL_COMM = serial.Serial('/dev/ttyUSB0', 9600)
COUNTER = 0
SUBSCRIPTIONS = []
NOTIFICATIONS = Queue()


class ServerSentEvent(object):
    """SSE "protocol" is described here: http://mzl.la/UPFyxY
    """

    def __init__(self, data):
        self.data = data
        self.event = None
        self.id = None
        self.desc_map = {
            self.data : "data",
            self.event : "event",
            self.id : "id"
        }

    def encode(self):
        if not self.data:
            return ""
        lines = [
            "%s: %s" % (v, k)
            for k, v in self.desc_map.iteritems() if k
        ]
        return "%s\n\n" % "\n".join(lines)


@APP.route('/')
def index():
    return render_template('index.html')


@APP.route('/debug')
def debug():
    return "Currently %d subscriptions" % len(SUBSCRIPTIONS)


@APP.route('/stream')
def particle_stream():
    def particles():
        q = Queue()
        SUBSCRIPTIONS.append(q)
        client_id = uuid4()
        msg = 'Client {0} connected!'.format(client_id)
        print(msg)
        NOTIFICATIONS.put(msg)
        try:
            while True:
                result = q.get()
                ev = ServerSentEvent(str(result))
                yield ev.encode()
        except GeneratorExit: # Or maybe use flask signals
            msg = 'Client {0} disconnected!'.format(client_id)
            print(msg)
            NOTIFICATIONS.put(msg)
            SUBSCRIPTIONS.remove(q)
    return Response(particles(), mimetype="text/event-stream")


def notify():
    global COUNTER

    while True:
        try:
            message = NOTIFICATIONS.get(block=False)
            for sub in SUBSCRIPTIONS[:]:
                sub.put(message)
        except Empty:
            pass

        COUNTER += 1
        message = SERIAL_COMM.readline()
        message = message.strip()
        for sub in SUBSCRIPTIONS[:]:
            sub.put("{0}: {1}".format(COUNTER, message))
        gevent.sleep(0.001)


if __name__ == "__main__":
    APP.debug = True
    server = WSGIServer(("", 5000), APP)
    gevent.spawn(notify)
    server.serve_forever()
