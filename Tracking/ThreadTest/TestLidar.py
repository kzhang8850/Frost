import time
import serial
import binascii
import pygame
import sys
import math
import cv2
import numpy as np
import os
from threading import Thread

ser = serial.Serial()
ser.port='/dev/ttyACM0'
ser.baudrate=115200
ser.parity=serial.PARITY_NONE
#ser.stopbits=serial.STOPBITS_ONE,
#bytesize=serial.SEVENBITS
ser.timeout = 1
ser.xonxoff = False     #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
ser.writeTimeout = 2     #timeout for write
ser.open()

# inSync = False
# sync = 0
# bytesRead = 0
# j = 0
# counter = 0
# dataReady = False
# dataArray = []

angle_1 = 340;
angle_2 = 70;
est_dist = 500;
est_threshold = 30;

class LidarView(object):
    def __init__(self, screen, model):
        self.screen = screen
        self.model = model
        self.center = model.center
        self.scaling = .3
    def draw(self, dataArray):
        self.screen.fill(pygame.Color('grey'))

        for i, obj in enumerate(dataArray):
            if obj != None:
                if angle_1 > angle_2:
                    if (angle_2 <= obj[0]<= angle_1):
                        dot_color = pygame.Color('red')
                    else:
                        dot_color = pygame.Color('green')
                else:
                    if (angle_1 <= obj[0]<= angle_2):
                        dot_color = pygame.Color('green')
                    else:
                        dot_color = pygame.Color('red')

                x = int((obj[1]+3)*math.cos(obj[0]*math.pi/180)*self.scaling)
                y = int((obj[1]+3)*math.sin(obj[0]*math.pi/180)*self.scaling)
                if (obj[1] > (est_dist - est_threshold) and obj[1] < (est_dist + est_threshold) and dot_color==pygame.Color('green')):
                    pygame.draw.circle(self.screen, pygame.Color('blue'), (self.center[0] - x, self.model.height -(self.center[1] - y)), 2)
                else:
                    pygame.draw.circle(self.screen, dot_color, (self.center[0] - x, self.model.height -(self.center[1] - y)), 2)
                #print (x,y)

        line1_x_pos = int(1000*math.cos(angle_1*math.pi/180))
        line1_y_pos = int(1000*math.sin(angle_1*math.pi/180))
        line2_x_pos = int(1000*math.cos(angle_2*math.pi/180))
        line2_y_pos = int(1000*math.sin(angle_2*math.pi/180))

        pygame.draw.circle(self.screen, pygame.Color('yellow'), (self.center[0], self.model.height - (self.center[1])), 3)

        pygame.draw.line(self.screen, pygame.Color('green'), (self.center[0], self.model.height - (self.center[1])),(self.center[0]-line1_x_pos, self.model.height - (self.center[1]-line1_y_pos)),1)
        pygame.draw.line(self.screen, pygame.Color('green'), (self.center[0], self.model.height - (self.center[1])),(self.center[0]-line2_x_pos, self.model.height - (self.center[1]-line2_y_pos)),1)

        if angle_1>angle_2:
            pygame.draw.arc(self.screen, pygame.Color('blue'), (self.center[0]-((est_dist+est_threshold)*self.scaling), self.model.height - (self.center[1])-((est_dist+est_threshold)*self.scaling),(est_dist+est_threshold)*2*self.scaling,(est_dist+est_threshold)*2*self.scaling),(angle_1-180)*(math.pi/180.),(angle_2+180)*(math.pi/180))
            pygame.draw.arc(self.screen, pygame.Color('blue'), (self.center[0]-((est_dist-est_threshold)*self.scaling), self.model.height - (self.center[1])-((est_dist-est_threshold)*self.scaling),(est_dist-est_threshold)*2*self.scaling,(est_dist-est_threshold)*2*self.scaling),(angle_1-180)*(math.pi/180.),(angle_2+180)*(math.pi/180))
        else:
            pygame.draw.arc(self.screen, pygame.Color('blue'), (self.center[0]-((est_dist+est_threshold)*self.scaling), self.model.height - (self.center[1])-((est_dist+est_threshold)*self.scaling),(est_dist+est_threshold)*2*self.scaling,(est_dist+est_threshold)*2*self.scaling),(angle_1+180)*(math.pi/180.),(angle_2+180)*(math.pi/180))
            pygame.draw.arc(self.screen, pygame.Color('blue'), (self.center[0]-((est_dist-est_threshold)*self.scaling), self.model.height - (self.center[1])-((est_dist-est_threshold)*self.scaling),(est_dist-est_threshold)*2*self.scaling,(est_dist-est_threshold)*2*self.scaling),(angle_1+180)*(math.pi/180.),(angle_2+180)*(math.pi/180))

        pygame.display.update()
class LidarModel(object):
    def __init__(self):
        self.width = 1000
        self.height = 1000
        self.center = (self.width/2, self.height/2)
        self.size = (self.width, self.height)

class LidarThread(Thread):
    def __init__(self, queue, stop, model, screen, view):
        super(LidarThread, self).__init__()
        self.queue = queue
        self.stop_event = stop
        self.model = model
        self.screen = screen
        self.view = view
        self.inSync = False
        self.sync = 0
        self.bytesRead = 0
        self.j = 0
        self.counter = 0
        self.dataReady = False
        self.dataArray = []
    def run(self):
        print 'also running!'
        while not self.stop_event.is_set():

            bytesRead = ser.inWaiting()
            data = ser.read()
            if(len(data) > 0):
                #print(sync)
                if ((ord(data) == 0xCC) and (self.sync == 0)) :
                    self.sync+=1
                elif ((ord(data) == 0xDD) and (self.sync == 1)) :
                    self.sync+=1
                elif ((ord(data) == 0xEE) and (self.sync == 2)) :
                    self.sync+=1
                elif ((ord(data) == 0xFF) and (self.sync == 3)) :
                    self.sync+=1
                else:
                    self.sync = 0

                #print(inSync)
                if(self.inSync):
                    if(self.j == 0):
                        #print("HighBit:")
                        #print(ord(data)<<8)
                        tempAngle = ord(data) << 8
                        self.j += 1
                        self.dataReady = False
                    elif(self.j == 1):
                        #print("LowBit:")
                        #print(ord(data))
                        angle = tempAngle + ord(data)
                        self.j += 1
                        self.dataReady = False
                    elif(self.j == 2):
                        tempDistance = ord(data) << 8
                        self.j += 1
                        self.dataReady = False
                    elif(self.j == 3):
                        distance = tempDistance + ord(data)
                        self.j = 0
                        self.dataReady = True
                    if(self.sync == 0 and self.dataReady):
                        self.dataArray.append((angle, distance))
                        #print(angle , distance)
                        #time.sleep()

                if (self.sync == 4):
                    self.inSync = True
                    self.counter += 1
                    self.view.draw(self.dataArray)
                    self.dataArray = []
        ser.close()