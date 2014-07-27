import storm

import base64
import json
import numpy as np

from modules import Tracking


frame_size = (360, 640)
frame_size_bytes = frame_size[0] * frame_size[1]

# this is the class we interface with storm. This will process the incoming messages by decoding them,
# do the image processing and create a command and emit it.
class TrackingBolt(storm.Bolt):
    def __init__(self):
        self.tracking = Tracking.Tracking()

    # this method will be called by storm when there is an incoming frame
    def process(self, tup):
        frame =  base64.b64decode(tup.values[0])
        original_time = tup.values[1]

        im = np.frombuffer(frame, count=frame_size_bytes, dtype=np.uint8)
        im = im.reshape((frame_size[0], frame_size[1], 3))
        frame = Tracking.ImageFrame(None, im)
        targets = self.tracking.do(frame)

        storm.emit([targets, original_time])

TrackingBolt().run()
