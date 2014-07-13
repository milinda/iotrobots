import storm
from modules.TrackingSensorModule import TrackingSensorModule

import time
import sys
import threading
import base64
import json
import numpy as np
import threading
from threading  import Thread
import Queue
import ctypes
from subprocess import PIPE, Popen
from StringIO import StringIO

lock = threading.RLock()

class SplitSentenceBolt(storm.BasicBolt):
    def __init__(self):
        self.trackingSensorModule = TrackingSensorModule()
        self.p = Popen(["nice", "-n", "15", "avconv", "-i", "-",
                        "-probesize", "2048", "-flags", "low_delay", "-f",
                        "rawvideo", "-pix_fmt", 'rgb24', "-"],
                       stdin=PIPE, stdout=PIPE, stderr=open('/dev/null', 'w'),
                       bufsize=0, preexec_fn=set_death_signal_int)
        t = Thread(target=self.enqueue_output, args=(self.p.stdout, (360, 640)))
        t.daemon = True
        t.start()

        self.q = Queue.Queue()

        self.time_queue = Queue.Queue()

        # counting the tuples emitted
        self.emit_count = 0
        self.tuple_count = 0
        self.time_removed = 0

        send_thread = Thread(target=self.send_queue)
        send_thread.daemon = True
        send_thread.start()

    def process(self, tup):
        frame =  base64.b64decode(tup.values[0])
        self.p.stdin.write(frame)
        time = tup.values[1]
        self.time_queue.put(time)
        self.tuple_count += 1

        # if self.time_queue.qsize() >= 10 and self.tuple_count > 130 and not self.time_removed:
        #     for x in range(0, 9):
        #         self.time_queue.get()
        #     self.time_removed = 1
        #     storm.log("done................")
        #
        # while not self.q.empty():
        #     msg = self.q.get()
        #     time = self.time_queue.get()
        #     self.emit_count += 1
        #     storm.emit([msg, time])
        #
        # storm.log("EC: " + str(self.emit_count) + " TC: " + str(self.tuple_count) + " MC: " + str(self.q.qsize()) + " TiC: " + str(self.time_queue.qsize()))

    def send_queue(self):
        # storm.log("hello")
        while 1:
            with lock :
                if self.time_queue.qsize() >= 10 and self.tuple_count > 130 and not self.time_removed:
                    for x in range(0, 8):
                        self.time_queue.get()
                    self.time_removed = 1

                storm.log("hello1")
            msg = self.q.get()
            time = self.time_queue.get()
            self.emit_count += 1

            with lock :
                storm.log("hello2")
                storm.emit([msg, time])
                storm.log("EC: " + str(self.emit_count) + " TC: " + str(self.tuple_count) + " MC: " + str(self.q.qsize()) + " TiC: " + str(self.time_queue.qsize()))

    def enqueue_output(self, out, frame_size):
        frame_size_bytes = frame_size[0] * frame_size[1] * 3

        while True:
            buffer_str = out.read(frame_size_bytes)
            im = np.frombuffer(buffer_str, count=frame_size_bytes, dtype=np.uint8)
            im = im.reshape((frame_size[0], frame_size[1], 3))
            circles = self.trackingSensorModule.detectCircles(im)

            message = {}

            if circles is not None:
                if circles.size == 3:
                    position = self.trackingSensorModule.calculatePosition(circles)
                    message["control"] = {"position" : [position[0], position[1]]}
                else:
                    message["control"] = {"hover" : "true1"}
            else:
                message["control"] = {"hover" : "true2"}

            io = StringIO()
            json.dump(message, io)
            self.q.put(io.getvalue())
            # storm.log("hello")
            # storm.emit([io.getvalue(), str(10000)])
            #storm.emit([io.getvalue()])

# Logic for making ffmpeg terminate on the death of this process
def set_death_signal(signal):
    libc = ctypes.CDLL('libc.so.6')
    PR_SET_DEATHSIG = 1
    libc.prctl(PR_SET_DEATHSIG, signal)

def set_death_signal_int():
    if sys.platform != 'darwin':
        SIGINT = 2
        SIGTERM = 15
        set_death_signal(SIGINT)

SplitSentenceBolt().run()