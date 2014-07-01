import cv2
import time
import sys
import numpy as np
import threading
from lib import libardrone

if __name__ == "__main__":
    drone = libardrone.ARDrone2()
    drone.set_speed(0.2)
    time.sleep(1)
    drone.takeoff()
    drone.halt()
