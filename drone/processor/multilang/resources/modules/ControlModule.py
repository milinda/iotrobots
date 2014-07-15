'''
@author: oliver lewis May 28, 2014
'''

from lib import libardrone
from modules.Planning import Planning
from modules.Tracking import Tracking
import datetime
import pika
import json
import cv2

command = {10:'takeoff', ord('l'):'land', ord(' '):'hover', ord('w'):'move_forward', ord('s'):'move_backward', ord('a'):'move_left', ord('d'):'move_right', 81:'turn_left', 82:'move_up',83:'turn_right',84:'move_down',8:'emergency'}


class ImageFrame:
    def __init__(self,header,pixels):
        self.header = header
        self.pixels = pixels




        
class Display():
    def __init__(self, imgFrame):
        self.imgFrame = imgFrame
        #self.fourcc = cv2.cv.CV_FOURCC(*'XVID')  
        #self.video_writer = cv2.VideoWriter("variousobjs4.avi", self.fourcc, 30, (640, 360))
    
    def do(self, targets):
        self.doCircles(targets)
        self.displayFrame()
        
    def doCircles(self, targets):
        for target in targets:
            cv2.circle(self.imgFrame.pixels,(int(target.x),int(target.y)),int(target.w/2),(0,0,255))
            cv2.line(self.imgFrame.pixels, self.getCenter(),(int(target.x),int(target.y)),(255,0,0))
            #cv2.putText(frame, "Radius:  "+str(circle[2]), (int(640/2), int(360/2)), 1, 1, (255,255,255))
            #cv2.putText(frame, "   Altitude:  "+str(altitude),(int(640/2.18), int(360/2.18)), 1, 1, (255,255,255))

    def displayFrame(self):
        #self.video_writer.write(self.imgFrame.pixels)
        cv2.imshow("Drone Camera",cv2.cvtColor(self.imgFrame.pixels, cv2.COLOR_BGR2RGB))
        
    def getCenter(self):
        width = 640
        height = 360
        center = (width/2,height/2)
        return center
        
        

class DroneNavData:
    def __init__(self, drone):
        self.navData = drone.get_navdata()[0]
        


class ControlModule():
    def __init__(self, drone):
        self.drone = drone
        self.auto_control = False
        #self.connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost', port=5672))
        #self.channel = self.connection.channel()
        #self.channel.exchange_declare(exchange="drone", exchange_type="direct", passive=False)
           

    def sendCommand(self, command):
        self.drone.apply_command(command)
        
    def sendPosition(self, x_pos, y_pos):
        self.drone.at(libardrone.at_pcmd, True, x_pos, y_pos, 0, 0)
        
    def hover(self):
        self.drone.hover()
        
    def stopDrone(self):
        self.drone.land()
        self.drone.halt()
        
    def autoControlDroneCloud(self, circles):
        method_frame, header_frame, body = self.channel.basic_get('control')
        if method_frame:
            self.channel.basic_ack(method_frame.delivery_tag)
            d = json.loads(body)
            print body
            #print str("position" in d["control"])
            if "position" in d["control"]:
                #print d["control"]["position"][0], d["control"]["position"][1]
                self.sendPosition(d["control"]["position"][0], d["control"]["position"][1])
            else:
                self.hover()
        else:
            self.hover()
    
    def completeController(self):
        try:
            #f = open('workfile.txt', 'w')
            d = {}
            while True:                               
                
                #print frame
                #f.write(str(datetime.datetime.now()))
                #f.write("\n")
                #str_alt = self.drone.get_navdata()[0]["altitude"]
                #f.write(str(str_alt))
                #f.write("\n")
                
                frame = self.drone.get_image()
                header = "valueHeader"   
                imgFrame = ImageFrame(header,frame)
                
                tracking = Tracking()
                targets = tracking.do(imgFrame)
                
                display = Display(imgFrame)
                display.do(targets)
                
                if self.auto_control:
                    planning = Planning()
                    droneNavData = DroneNavData(self.drone)
                    droneCommand = planning.do(droneNavData, targets)
                    droneCommand.do(self.drone) 
                k = cv2.waitKey(1) & 0xFF
                if k in command:
                    self.sendCommand(command[k])
                elif k == ord('c'):
                    self.auto_control = True
                elif k == ord('x'):
                    self.auto_control = False
                elif k == ord('q'):
                    
                    break
        finally:
            self.stopDrone()       
                
