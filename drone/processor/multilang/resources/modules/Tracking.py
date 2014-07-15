import cv2
import numpy as np

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