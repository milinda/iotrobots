import storm

import time
import base64
import json
import numpy as np
from StringIO import StringIO

from modules import Tracking

frame_size = (360, 640)
frame_size_bytes = frame_size[0] * frame_size[1] * 3

# this is the class we interface with storm. This will process the incoming messages by decoding them,
# do the image processing and create a command and emit it.
class TrackingBolt(storm.BasicBolt):
    def __init__(self):
        self.tracking = Tracking.Tracking()

    # this method will be called by storm when there is an incoming frame
    def process(self, tup):
        start_time = int(round(time.time() * 1000))
        frame =  base64.b64decode(tup.values[0])
        original_time = tup.values[1]
        sensorId = tup.values[2]

        im = np.frombuffer(frame, count=frame_size_bytes, dtype=np.uint8)
        im = im.reshape((frame_size[0], frame_size[1], 3))
        frame = Tracking.ImageFrame(None, im)
        targets = self.tracking.do(frame)
        end_time = int(round(time.time() * 1000))

        storm.log("Tracking Processing time: " + str(end_time - start_time))

        targets_message = []
        for target in targets:
            targets_message.append({'found': 1, 'x': str(target.x), 'y': str(target.y), 'h': str(target.h), 'w': str(target.w)})

        storm.emit([targets_message, original_time, sensorId])

TrackingBolt().run()
