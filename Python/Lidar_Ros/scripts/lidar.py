#!/usr/bin/env python
#################################################################################################################################
"""
LIDAR Module - Acts as Frost's Depth Perception

Establishes a connection to the LIDAR sensor's Arduino
Receives data from said Arduino and then parses it into usable form, namely hexadeicmal to angles and distances

Written by Cedric Kim
"""
################################################################################################################################

import rospy
import time
import serial
import sys
import math
import cv2
import numpy as np
import os
from frost_lidar.msg import Polar
from frost_lidar.msg import Polar_Array

class LidarSerial(object):
    def __init__(self):
        """
        Serial stuff
        """
        #for serial input from arduino for LIDAR
        self.ser = serial.Serial()
        self.ser.port='/dev/ttyACM0'
        self.ser.baudrate=115200
        self.ser.timeout = 1
        self.ser.writeTimeout = 2     #timeout for write
        self.ser.open()

class Lidar(object):
    """
    the main LIDAR object, which receives and parses data from the LIDAR
    """
    def __init__(self, serial):
        rospy.init_node('lidar')
        self.pub = rospy.Publisher('lidar_data', Polar_Array, queue_size = 1)
        self.ser = serial.ser
        self.inSync = False
        self.sync = 0
        self.bytesRead = 0
        self.j = 0
        self.counter = 0
        self.dataReady = False
        self.dataArray = Polar_Array()
        self.tempAngle = 0
        self.tempDistance = 0
        self.angle = 0
        self.distance = 0

    def run(self):
        """
        reads data from LIDAR's serial port and parses it into meaningful data
        """
        #self.bytesRead = self.ser.inWaiting()
        data = self.ser.read()
        if(len(data) > 0):
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

            if(self.inSync):
                if(self.j == 0):
                    self.tempAngle = ord(data) << 8
                    self.j += 1
                    self.dataReady = False
                elif(self.j == 1):
                    self.angle = self.tempAngle + ord(data)
                    self.j += 1
                    self.dataReady = False
                elif(self.j == 2):
                    self.tempDistance = ord(data) << 8
                    self.j += 1
                    self.dataReady = False
                elif(self.j == 3):
                    self.distance = self.tempDistance + ord(data)
                    self.j = 0
                    self.dataReady = True
                if(self.sync == 0 and self.dataReady):
                    msg = Polar()
                    msg.theta = self.angle
                    msg.r = self.distance
                    #print type(data)
                    #data.Polar_Array.append(msg)
                    self.dataArray.Polar_Array.append(msg)

        if (self.sync == 4):
            #print data
            self.inSync = True
            self.counter += 1
            #data = self.dataArray
            #print self.dataArray
            self.pub.publish(self.dataArray)
            #self.dataArray = []
            self.dataArray = Polar_Array()


if __name__ == '__main__':
    serial = LidarSerial()
    lidar = Lidar(serial)
    #r = rospy.Rate(100)
    while not rospy.is_shutdown():
        lidar.run()
        #r.sleep()
    serial.ser.close()