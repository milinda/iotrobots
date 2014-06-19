import storm

from lib import libardrone
from lib import h264decoder
import time
import sys
import threading
import base64
import json
import numpy as np
import ctypes
from subprocess import PIPE, Popen
from StringIO import StringIO

from modules.TrackingSensorModule import TrackingSensorModule

class SplitSentenceBolt(storm.BasicBolt):
    def __init__(self):
        self.trackingSensorModule = TrackingSensorModule()
        self.p = Popen(["nice", "-n", "15", "avconv", "-i", "-",
                   "-probesize", "2048", "-flags", "low_delay", "-f",
                   "rawvideo", "-pix_fmt", 'rgb24', "-"],
                  stdin=PIPE, stdout=PIPE, stderr=open('/dev/null', 'w'),
                  bufsize=0, preexec_fn=set_death_signal_int)
#        self.decoder = h264decoder.H264Decoder((360, 640, 3))

    def process(self, tup):
        base64Frame = [elem.encode("hex") for elem in tup.values[0]]

        frame =  base64.b64decode(tup.values[0])
        frame_bytes = [elem.encode("hex") for elem in frame]
        self.p.stdin.write(frame)
        frame_size=(360, 640)
        frame_size_bytes = frame_size[0] * frame_size[1] * 3
        storm.emit(["section1"])
        buffer_str = self.p.stdout.read(frame_size_bytes)
        storm.emit(["section2"])
        im = np.frombuffer(buffer_str, count=frame_size_bytes, dtype=np.uint8)
        im = im.reshape((frame_size[0], frame_size[1], 3))
        storm.emit(["section3"])
        circles = self.trackingSensorModule.detectCircles(im)
        storm.emit(["section4"])
        message = {}

        if circles is not None:
            if circles.size == 3:
                position = self.trackingSensorModule.calculatePosition(circles)
                message["control"] = {"position" : [position[0], position[1]]}
            else:
                message["control"] = {"hover" : "true"}
        else:
            message["control"] = {"hover" : tup.values[0]}

        io = StringIO()
        json.dump(message, io)
        storm.emit([io.getvalue()])

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