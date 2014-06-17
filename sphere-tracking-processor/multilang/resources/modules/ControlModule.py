'''
@author: oliver lewis May 28, 2014
'''

from lib import libardrone
import cv2
from modules.TrackingSensorModule import TrackingSensorModule

command = {10:'takeoff', ord('l'):'land', ord(' '):'hover', ord('w'):'move_forward', ord('s'):'move_backward', ord('a'):'move_left', ord('d'):'move_right', 81:'turn_left', 82:'move_up',83:'turn_right',84:'move_down',8:'emergency'}
class ControlModule():
    def __init__(self, drone, trackingSensorModule):
        self.drone = drone
        self.trackingSensorModule = trackingSensorModule
        self.auto_control = False
           

    def sendCommand(self, command):
        self.drone.apply_command(command)
        
    def sendPosition(self, x_pos, y_pos):
        self.drone.at(libardrone.at_pcmd, True, x_pos, y_pos, 0, 0)
        
    def hover(self):
        self.drone.hover()
        
    def stopDrone(self):
        self.drone.land()
        self.drone.halt()
        
    def autoControlDrone(self, circles):
        if circles != None:
            if(circles.size == 3):
                position = self.trackingSensorModule.calculatePosition(circles)
                self.sendPosition(position[0], position[1])
            else:
                self.hover()
        else:
            self.hover()
            
    def completeController(self):
        try:
            while True:                               
                frame = self.drone.get_image()
                '''detect_circles'''
                circles = self.trackingSensorModule.detectCircles(frame)
                if circles != None:
                    frame = self.trackingSensorModule.drawTrackingImages(frame, circles)
                
                self.trackingSensorModule.displayDroneCamera(frame)
                
                '''(x,y) = calculate_positions()'''
                if self.auto_control:
                    self.autoControlDrone(circles)                
                
                k = cv2.waitKey(1) & 0xFF
                print k, "################"
                if k in command:
                    self.sendCommand(command[k])
                elif k == ord('c'):
                    self.auto_control = True
                elif k == ord('x'):
                    self.auto_control = False
        finally:
            self.stopDrone()       
                
