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

import ControlModule

lock = threading.RLock()

class DroneFrameProcessBolt(storm.Bolt):
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
        self.tuple_count = 0
        self.time_removed = 0
        self.diff = []

        # the control modules
        self.tracking = ControlModule.Tracking()
        self.planing = ControlModule.Planning()

        send_thread = Thread(target=self.send_queue)
        send_thread.daemon = True
        send_thread.start()

    def process(self, tup):
        frame =  base64.b64decode(tup.values[0])
        self.p.stdin.write(frame)
        t = tup.values[1]
        curtime = int(round(time.time() * 1000))
        self.local_time_queue.put(curtime)
        self.time_queue.put(t)
        self.tuple_count += 1
        with lock:
            storm.ack(tup)

    def send_queue(self):
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
            t = self.time_queue.get()
            t2 = self.local_time_queue.get()
            current_time = int(round(time.time() * 1000))
            self.emit_count += 1

            with lock :
                storm.emit([msg, t])
                storm.log("EC: " + str(self.emit_count) + " TC: " + str(self.tuple_count) + " MC: " +
                          str(self.frame_queue.qsize()) + " TiC: " + str(self.time_queue.qsize()) + " LAT: " + str(current_time - t2))
                self.diff.append(self.tuple_count - self.emit_count)

    def enqueue_output(self, out, frame_size):
        frame_size_bytes = frame_size[0] * frame_size[1] * 3

        while True:
            buffer_str = out.read(frame_size_bytes)
            im = np.frombuffer(buffer_str, count=frame_size_bytes, dtype=np.uint8)
            im = im.reshape((frame_size[0], frame_size[1], 3))

            frame = ControlModule.ImageFrame(None, im)

            targets = self.tracking.do(frame)
            command = self.planing.do(None, targets)

            message = {}
            if command is not None:
                message["control"] = {"position" : [command.tiltx, command.tilty]}
            else:
                message["control"] = {"hover" : "true"}

            io = StringIO()
            json.dump(message, io)
            self.frame_queue.put(io.getvalue())

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

DroneFrameProcessBolt().run()