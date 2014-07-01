'''
Created on Apr 29, 2014

@author: shahensha
@author: oliver lewis May 28, 2014
'''
import time
import sys
import threading
from resources.lib import libardrone
from resources.modules import TrackingSensorModule, ControlModule


class VideoController(threading.Thread):
    
    def __init__(self, drone, sensorModule):
        threading.Thread.__init__(self)
        self.drone = drone  
        self.sensorModule = sensorModule      
        
    def run(self):
        '''test commit to git'''
        controlModule = ControlModule(self.drone, self.sensorModule)
        controlModule.completeController()

'''This class is complete for operation'''            
class ManualController(threading.Thread):
    
    def __init__(self, drone):
        threading.Thread.__init__(self)
        self.drone = drone
        
    def run(self):
        try:
            while True:
                c = sys.stdin.read(1)
                c = c.lower()
                print "Got character", c
                if c == 'a':
                    drone.move_left()
                if c == 'd':
                    drone.move_right()
                if c == 'w':
                    drone.move_forward()
                if c == 's':
                    drone.move_backward()
                if c == ' ':
                    drone.land()
                if c == '\n':
                    drone.takeoff()
                if c == 'q':
                    drone.turn_left()
                if c == 'e':
                    drone.turn_right()
                if c == '1':
                    drone.move_up()
                if c == '2':
                    drone.hover()
                if c == '3':
                    drone.move_down()
                if c == 't':
                    drone.reset()
                if c == 'x':
                    drone.hover()
                if c == 'y':
                    drone.trim()
                if c == 'b':
                    break
        finally:
            drone.halt()
            
    
if __name__ == "__main__":
    '''Get the drone object'''
    drone = libardrone.ARDrone2()
    trackingSensorModule = TrackingSensorModule()
    '''configure drone: To be used to set all the manual configurations of the drone object'''
    drone.configureDrone()
        
    time.sleep(1)
    '''Sending the drone to the controller'''
    controller = VideoController(drone, trackingSensorModule)
    '''Uncomment below line to use the manual controller'''
    #controller = ManualController(drone)
    
    controller.start()
