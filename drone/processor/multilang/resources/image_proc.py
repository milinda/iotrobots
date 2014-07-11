import storm
from modules.TrackingSensorModule import TrackingSensorModule

import time
import sys
import threading
import base64
import json
import numpy as np
from threading  import Thread
import Queue
import ctypes
from subprocess import PIPE, Popen
from StringIO import StringIO


class SplitSentenceBolt(storm.BasicBolt):
    def __init__(self):
        self.trackingSensorModule = TrackingSensorModule()
        self.p = Popen(["nice", "-n", "15", "avconv", "-i", "-",
                   "-probesize", "2048", "-flags", "low_delay", "-f",
                   "rawvideo", "-pix_fmt", 'rgb24', "-"],
                  stdin=PIPE, stdout=PIPE, stderr=open('/dev/null', 'w'),
                  bufsize=0, preexec_fn=set_death_signal_int)
        t = Thread(target=self.enqueue_output, args=(self.p.stdout, (360, 640)))
        t.daemon = True # thread dies with the program
        self.q = Queue.Queue()
        self.time_queue = Queue.Queue()
        self.tcount = 0
        self.emitcount = 0
        self.started_decode = 0
        t.start()

    def process(self, tup):
        frame =  base64.b64decode(tup.values[0])

        if self.tcount > 8:
            time = tup.values[1]
            self.time_queue.put(time)

        self.p.stdin.write(frame)
        self.tcount += 1
        while not self.q.empty():
            # if not self.started_decode:
            #     storm.log(("size of time queue ****** " + str(self.time_queue.qsize())))
            #     if self.time_queue.qsize() > 2:
            #         while self.time_queue.qsize() != 3:
            #             self.time_queue.get()
            #     self.started_decode = 1

            list = []
            time = self.time_queue.get()
            list.append(self.q.get())
            list.append(time)

            storm.log(("size of time queue " + str(self.time_queue.qsize())))
            storm.log(("size of message queue " + str(self.q.qsize())))

            # if (self.time_queue.qsize() != self.q.qsize()):
            #     with self.q.mutex:
            #         self.q.queue.clear()
            #         while not self.time_queue.empty():
            #             self.time_queue.get()
            self.q.queue.clear()
            self.emitcount += 1
            storm.log(("emit count " + str(self.emitcount)))
            storm.log(("tuple count " + str(self.tcount)))
            storm.emit(list)

    def enqueue_output(self, out, frame_size):
        frame_size_bytes = frame_size[0] * frame_size[1] * 3

        #print "frame: ", frame_size_bytes
        while True:
            #self.q.put("before")
            buffer_str = out.read(frame_size_bytes)
            #self.q.put("after")
            #message["control"] = {"hover" : "true1"}
            #storm.emit(["{topic1:section4}"])
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