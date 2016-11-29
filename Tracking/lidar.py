import time
import serial
import binascii
import pygame
import sys
import math
import cv2
import numpy as np
import body_detection

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
inSync = False

sync = 0
bytesRead = 0
j = 0
angle = 0
distance = 0
counter = 0
angle = 0
counter = 0
insync = False
dataReady = False
dataArray = []

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
        angle = 500
        repeatedAngles = 0
        # for i, obj in enumerate(dataArray):
            
        #     if(angle == obj[0]):
        #         repeatedAngles += 1
        #         repeatedAngles = 0
        #     elif(repeatedAngles > 0):
        #         for j in range(0, repeatedAngles):
        #             #print (1.0/(repeatedAngles+1))
        #             dataArray[i-j] = (dataArray[i-j][0] - 1.0/(repeatedAngles+1), dataArray[i-j][1])
        #             #print(dataArray[i-j])
        #             repeatedAngles = 0
        #     angle = obj[0]
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
                    pygame.draw.circle(screen, pygame.Color('blue'), (self.center[0] - x, model.height -(self.center[1] - y)), 2)
                else:
                    pygame.draw.circle(screen, dot_color, (self.center[0] - x, model.height -(self.center[1] - y)), 2)
                #print (x,y)

        line1_x_pos = int(1000*math.cos(angle_1*math.pi/180))
        line1_y_pos = int(1000*math.sin(angle_1*math.pi/180))
        line2_x_pos = int(1000*math.cos(angle_2*math.pi/180))
        line2_y_pos = int(1000*math.sin(angle_2*math.pi/180))

        pygame.draw.circle(screen, pygame.Color('yellow'), (self.center[0], model.height - (self.center[1])), 3)

        pygame.draw.line(screen, pygame.Color('green'), (self.center[0], model.height - (self.center[1])),(self.center[0]-line1_x_pos, model.height - (self.center[1]-line1_y_pos)),1)
        pygame.draw.line(screen, pygame.Color('green'), (self.center[0], model.height - (self.center[1])),(self.center[0]-line2_x_pos, model.height - (self.center[1]-line2_y_pos)),1)

        if angle_1>angle_2:
            pygame.draw.arc(screen, pygame.Color('blue'), (self.center[0]-((est_dist+est_threshold)*self.scaling), model.height - (self.center[1])-((est_dist+est_threshold)*self.scaling),(est_dist+est_threshold)*2*self.scaling,(est_dist+est_threshold)*2*self.scaling),(angle_1-180)*(math.pi/180.),(angle_2+180)*(math.pi/180))
            pygame.draw.arc(screen, pygame.Color('blue'), (self.center[0]-((est_dist-est_threshold)*self.scaling), model.height - (self.center[1])-((est_dist-est_threshold)*self.scaling),(est_dist-est_threshold)*2*self.scaling,(est_dist-est_threshold)*2*self.scaling),(angle_1-180)*(math.pi/180.),(angle_2+180)*(math.pi/180))
        else:
            pygame.draw.arc(screen, pygame.Color('blue'), (self.center[0]-((est_dist+est_threshold)*self.scaling), model.height - (self.center[1])-((est_dist+est_threshold)*self.scaling),(est_dist+est_threshold)*2*self.scaling,(est_dist+est_threshold)*2*self.scaling),(angle_1+180)*(math.pi/180.),(angle_2+180)*(math.pi/180))
            pygame.draw.arc(screen, pygame.Color('blue'), (self.center[0]-((est_dist-est_threshold)*self.scaling), model.height - (self.center[1])-((est_dist-est_threshold)*self.scaling),(est_dist-est_threshold)*2*self.scaling,(est_dist-est_threshold)*2*self.scaling),(angle_1+180)*(math.pi/180.),(angle_2+180)*(math.pi/180))

        pygame.display.update()
class LidarModel(object):
    def __init__(self):
        self.width = 1000
        self.height = 1000
        self.center = (self.width/2, self.height/2)
        self.size = (self.width, self.height)


if __name__ == '__main__':
    Bodies = body_detection.BodyDetector()

    pygame.init()
    model = LidarModel()
    screen = pygame.display.set_mode(model.size)
    view = LidarView(screen, model)

    while True:

        for event in pygame.event.get():
            if event.type == pygame.QUIT: sys.exit()
        bytesRead = ser.inWaiting()
        #print bytesRead
        #print bytesRead
        #if(bytesRead > 0):
        #print bytesRead
        #for i, item in enumerate(data):
        #    print(ord(item))
            #print 'test'
        while(bytesRead > 1):
            bytesRead = ser.inWaiting()
            data = ser.read()
            if(len(data) > 0):
                #print(sync)
                if ((ord(data) == 0xCC) and (sync == 0)) :
                    sync+=1
                elif ((ord(data) == 0xDD) and (sync == 1)) :
                    sync+=1
                elif ((ord(data) == 0xEE) and (sync == 2)) :
                    sync+=1
                elif ((ord(data) == 0xFF) and (sync == 3)) :
                    sync+=1
                else:
                    sync = 0

                #print(inSync)
                if(inSync):
                    if(j == 0):
                        #print("HighBit:")
                        #print(ord(data)<<8)
                        tempAngle = ord(data) << 8
                        j += 1
                        dataReady = False
                    elif(j == 1):
                        #print("LowBit:")
                        #print(ord(data))
                        angle = tempAngle + ord(data)
                        j += 1
                        dataReady = False
                    elif(j == 2):
                        tempDistance = ord(data) << 8
                        j += 1
                        dataReady = False
                    elif(j == 3):
                        distance = tempDistance + ord(data)
                        j = 0
                        dataReady = True
                    if(sync == 0 and dataReady):
                        dataArray.append((angle, distance))
                        #print(angle , distance)
                        #time.sleep()

                if (sync == 4):
                    inSync = True
                    counter += 1
                    view.draw(dataArray)
                    dataArray = []
       #if(sync == 4):
        #print("headsads")
        crowd = Bodies.find_bodies()

        # Targeter.track(crowd)

        k = cv2.waitKey(30) & 0xff

        if k == 27:

            break

    ser.close()

    Bodies.cam.release()
    cv2.destroyAllWindows()