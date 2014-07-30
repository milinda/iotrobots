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
class PlanningBolt(storm.BasicBolt):
    def __init__(self):
        self.planing = Planning.Planning()

    # this method will be called by storm when there is an incoming frame
    def process(self, tup):
        targets_message = tup.values[0]
        if targets_message:
            targets = []
            # json_object = json.loads(targets_message)
            for o in targets_message:
                t = Tracking.Target(o["found"], float(o["x"]), float(o["y"]), float(o["w"]), float(o["h"]))
                targets.append(t)

            original_time = tup.values[1]
            command = self.planing.do(None, targets)

            message = {"command": [command.tiltx, command.tilty, command.spin, command.velocity_z]}
            io = StringIO()
            json.dump(message, io)

            storm.emit([io.getvalue(), original_time])
        elif nav_data:
            storm.log(nav_data)

PlanningBolt().run()