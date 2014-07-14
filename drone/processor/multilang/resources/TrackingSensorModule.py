'''@author: oliver lewis May 28, 2014'''

import cv2
import numpy as np
from math import floor, sqrt

class TrackingSensorModule():
    def __init__(self):
        self.fourcc = cv2.cv.CV_FOURCC(*'XVID')  
        #self.video_writer = cv2.VideoWriter("variousobjs4.avi", self.fourcc, 30, (640, 360))
        
    def detectCircles(self, frame, altitude):
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray,(3,3),0)
        canny = cv2.Canny(gray,100,200)
        #min = int(14.99+(1.081*10**11/altitude**3.426)-2)
        #max = int(14.99+(1.081*10**11/altitude**3.426)+2)
        #min = int((25446.842/sqrt(altitude**2+(0.40835*altitude)**2))-2)
        #max = int((25446.842/altitude)+2)
        circles = cv2.HoughCircles(canny,3,1,20,param1=120,param2=10,minRadius=15,maxRadius=25)
        if circles != None:
            n = np.shape(circles)
            circles=np.reshape(circles,(n[1],n[2]))
            det_circle_count = circles.size/3
        return circles
    
    def drawTrackingImages(self, frame, circles, altitude):
        for circle in circles:
            cv2.circle(frame,(circle[0],circle[1]),circle[2],(0,0,255))
            #str_rad = "radius: ",circle[2]
            #f.write(str(str_rad))
            #f.write("\n\n")
            cv2.line(frame, self.getCenter(),(circle[0],circle[1]),(255,0,0))
            #cv2.putText(frame, "Radius:  "+str(circle[2]), (int(640/2), int(360/2)), 1, 1, (255,255,255))
            #cv2.putText(frame, "   Altitude:  "+str(altitude),(int(640/2.18), int(360/2.18)), 1, 1, (255,255,255))
        return frame
    
    def displayDroneCamera(self,frame):
        #self.video_writer.write(frame)
        cv2.imshow("Drone Camera",cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        
    
    
    def calculatePosition(self, circles):
        if circles.size == 3:
            x = circles[0][0]
            y = circles[0][1]
            target_y = self.getCenter()[1] - y
            target_x = x - self.getCenter()[0]
            kX = 1.0/320*0.1
            kY = -1.0/320*0.1
        return (kX*target_x, kY*target_y)

    def getCenter(self):
        width = 640
        height = 360
        center = (width/2,height/2)
        return center
        
