'''@author: oliver lewis May 28, 2014'''

import cv2
import numpy as np

class TrackingSensorModule():
    '''def __init__(self, frame):
        self.frame = frame'''
        
    def detectCircles(self, frame):
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray,(3,3),0)
        canny = cv2.Canny(gray,100,200)
        circles = cv2.HoughCircles(canny,3,1,20,param1=120,param2=10,minRadius=15,maxRadius=25)
        if circles != None:
            n = np.shape(circles)
            circles=np.reshape(circles,(n[1],n[2]))
            det_circle_count = circles.size/3
        return circles
    
    def drawTrackingImages(self, frame, circles):
        for circle in circles:
            cv2.circle(frame,(circle[0],circle[1]),circle[2],(0,0,255))
            cv2.line(frame, self.getCenter(),(circle[0],circle[1]),(255,0,0))
        return frame
    
    def displayDroneCamera(self,frame):
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
        
