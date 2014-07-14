'''
@author: oliver lewis May 28, 2014
'''

from lib import libardrone
import cv2
import TrackingSensorModule
import datetime
import pika
import json
import numpy as np

command = {10:'takeoff', ord('l'):'land', ord(' '):'hover', ord('w'):'move_forward', ord('s'):'move_backward', ord('a'):'move_left', ord('d'):'move_right', 81:'turn_left', 82:'move_up',83:'turn_right',84:'move_down',8:'emergency'}

class ImageFrame:
    def __init__(self,header,pixels):
        self.header = header
        self.pixels = pixels

class Target:
    def __init__(self,found,x,y,w,h):
        #x,y: Estimated position of target on ground plane, relative to drone (in meters)
        self.found = found
        self.x = x
        self.y = y
        self.w = w
        self.h = h

class DroneCommand:
    def __init__(self,tiltx,tilty,spin,velocity_z):
        self.tiltx = tiltx
        self.tilty = tilty
        self.spin = spin
        self.velocity_z = velocity_z

    def do(self, drone):
        print "Tilt X and Y: ",self.tiltx, self.tilty
        drone.at(libardrone.at_pcmd, True, self.tiltx, self.tilty, 0, 0)

class Display():
    def __init__(self, imgFrame):
        self.imgFrame = imgFrame

    def do(self, targets):
        self.doCircles(targets)
        self.displayFrame()

    def doCircles(self, targets):
        for target in targets:
            cv2.circle(self.imgFrame.pixels,(int(target.x),int(target.y)),int(target.w/2),(0,0,255))
            cv2.line(self.imgFrame.pixels, self.getCenter(),(int(target.x),int(target.y)),(255,0,0))

    def displayFrame(self):
        cv2.imshow("Drone Camera",cv2.cvtColor(self.imgFrame.pixels, cv2.COLOR_BGR2RGB))

    def getCenter(self):
        width = 640
        height = 360
        center = (width/2,height/2)
        return center

class Tracking():
    '''def __init__(self, frame):
        self.frame = frame'''
    def do(self, imageFrame):
        targets = []
        circles = self.doCircles(imageFrame)
        '''Appending circle targets to the target array'''
        if circles is not None:
            for circle in circles:
                targets.append(Target(True,circle[0],circle[1],circle[2]*2,circle[2]*2))
        return targets

    def doCircles(self, imageFrame):
        gray = cv2.cvtColor(imageFrame.pixels,cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray,(3,3),0)
        canny = cv2.Canny(gray,100,200)
        circles = cv2.HoughCircles(canny,3,1,20,param1=120,param2=10,minRadius=15,maxRadius=25)
        if circles != None:
            n = np.shape(circles)
            circles=np.reshape(circles,(n[1],n[2]))
            det_circle_count = circles.size/3
        return circles

class DroneNavData:
    def __init__(self, drone):
        self.navData = drone.get_navdata()[0]

class Planning():
    def do(self, droneNavData, targets):
        if(len(targets) == 1):
            if targets[0].found == True:
                x = targets[0].x
                y = targets[0].y
                kX = -(1.0/320*0.1)
                kY = -(-1.0/320*0.1)
                return DroneCommand(kX*x, kY*y, 0, 0)
            else:
                return DroneCommand(0,0,0,0)
        return DroneCommand(0,0,0,0)

class ControlModule():
    def __init__(self, drone, trackingSensorModule):
        self.drone = drone
        self.trackingSensorModule = trackingSensorModule
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
                '''detect_circles'''
                '''circles = self.trackingSensorModule.detectCircles(imgFrame.pixels, str_alt)
                if circles != None:
                    imgFrame.pixels = self.trackingSensorModule.drawTrackingImages(imgFrame.pixels, circles, str_alt)
                self.trackingSensorModule.displayDroneCamera(imgFrame.pixels)'''

                '''(x,y) = calculate_positions()'''
                frame = self.drone.get_image()
                header = "valueHeader"
                imgFrame = ImageFrame(header,frame)

                tracking = Tracking()
                targets = tracking.do(imgFrame)

                display = Display(imgFrame)
                display.do(targets)

                if self.auto_control:
                    #print "auto control started."
                    #self.autoControlDrone(circles)
                    planning = Planning()
                    droneNavData = DroneNavData(self.drone)
                    droneCommand = planning.do(droneNavData, targets)
                    droneCommand.do(self.drone)
                    #else:
                    #self.channel.basic_get('control')              

                k = cv2.waitKey(1) & 0xFF
                #print k, "################"
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
                
