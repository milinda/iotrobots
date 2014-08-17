import storm

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

from modules import Planning
from modules import Tracking

lock = threading.RLock()

# this is the class we interface with storm. This will process the incoming messages by decoding them,
# do the image processing and create a command and emit it.
class DecodeAndTrackingBolt(storm.Bolt):
    def __init__(self):
        self.p = Popen(["nice", "-n", "15", "avconv", "-i", "-",
                        "-probesize", "2048", "-flags", "low_delay", "-f",
                        "rawvideo", "-pix_fmt", 'rgb24', "-"],
                       stdin=PIPE, stdout=PIPE, stderr=open('/dev/null', 'w'),
                       bufsize=0, preexec_fn=set_death_signal_int)
        t = Thread(target=self.enqueue_output, args=(self.p.stdout, (360, 640)))
        t.daemon = True
        t.start()

        self.frame_queue = Queue.Queue()

        self.time_queue = Queue.Queue()
        self.local_time_queue = Queue.Queue()

        # counting the tuples emitted
        self.emit_count = 0
        # frames received
        self.tuple_count = 0
        # weather we have removed the times from the time queue
        self.time_removed = 0
        # used to keep the difference between frames recieved and messages emitted
        self.diff = []

        # the control modules
        self.tracking = Tracking.Tracking()
        self.planing = Planning.Planning()

        send_thread = Thread(target=self.emit_message)
        send_thread.daemon = True
        send_thread.start()

    # this method will be called by storm when there is an incoming frame
    def process(self, tup):
        frame =  base64.b64decode(tup.values[0])
        self.p.stdin.write(frame)
        t = tup.values[2]
        sensorId = tup.values[1]
        curtime = int(round(time.time() * 1000))
        self.local_time_queue.put(curtime)
        self.time_queue.put([t, sensorId])
        self.tuple_count += 1
        with lock:
            storm.ack(tup)

    # emit a message to storm, the message is a command in the json format
    def emit_message(self):
        while 1:
            if len(self.diff) > 100 and not self.time_removed:
                equal = 1
                cd = 0
                pd = 0
                d = 0
                for x in range(0, 99):
                    cd = self.diff[-1 - x] - self.diff[-2 - x]
                    if x == 0:
                        pd = cd
                    if pd != cd:
                        equal = 0
                    pd = cd
                    d = self.diff[-2 - x]

                if equal:
                    for x in range(0, d):
                        self.time_queue.get()
                        self.local_time_queue.get()
                    self.time_removed = 1

            msg = self.frame_queue.get()
            [t, sensorId] = self.time_queue.get()
            t2 = self.local_time_queue.get()
            current_time = int(round(time.time() * 1000))
            self.emit_count += 1

            # acquire the lock to avoid storm and the message processing thread do emit and ack at the same time
            with lock :
                storm.log("sensor id: " + str(sensorId))
                storm.emit([msg, sensorId, t])
                storm.log("EC: " + str(self.emit_count) + " TC: " + str(self.tuple_count) + " MC: " +
                          str(self.frame_queue.qsize()) + " TiC: " + str(self.time_queue.qsize()) + " LAT: " + str(current_time - t2))

            if not self.time_removed:
                self.diff.append(self.tuple_count - self.emit_count)

    # process the image frame and produce a DroneCommand
    def process_frame(self, buffer_str, frame_size):
        frame_size_bytes = frame_size[0] * frame_size[1] * 3

        im = np.frombuffer(buffer_str, count=frame_size_bytes, dtype=np.uint8)
        im = im.reshape((frame_size[0], frame_size[1], 3))
        frame = Tracking.ImageFrame(None, im)
        targets = self.tracking.do(frame)

        targets_message = []
        for target in targets:
            targets_message.append({'found': 1, 'x': str(target.x), 'y': str(target.y), 'h': str(target.h), 'w': str(target.w)})

        return targets_message

    # read the output of the decoder and process it
    def enqueue_output(self, out, frame_size):
        frame_size_bytes = frame_size[0] * frame_size[1] * 3

        while True:
            buffer_str = out.read(frame_size_bytes)
            message = self.process_frame(buffer_str, frame_size)

            self.frame_queue.put(message)

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

DecodeAndTrackingBolt().run()