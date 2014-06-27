'''
Created on Apr 7, 2014

@author: shahensha
'''
import libardrone
import time
import numpy as np
import cv2

drone = libardrone.ARDrone2()

try:
    drone.takeoff()
    time.sleep(5)
    drone.move_forward()
    time.sleep(1)
    drone.hover()
    image = drone.get_image()
    drone.land()
finally:
    drone.halt()
plt.imshow(image,interpolation='nearest')
plt.show()
print "The End"
