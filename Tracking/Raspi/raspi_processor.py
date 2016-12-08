import time
import serial
import binascii
import pygame
import sys
import math
import cv2
import numpy as np
import os


class Supervisor(object):
    """
    main class - instantiates all processing classes for LIDAR, Kinect, and SerialOut
    """
    def __init__(self, ser_out):
        self.view = LidarView()
        self.serial_out = SerialOut(ser_out)
        self.targeter = TargetLocator()

class SerialOut(object):
    """
    class that holds processing to push output data to the launcher for panning and shooting a set distance
    """
    def __init__(self, ser_out):
        self.ser_out = ser_out
        self.prev_time = time.time()
        self.time_to_arm = 3
        self.time_to_shoot = 6
        self.time_to_send_angle = .2
        self.prev_time_angle = time.time()
        self.arm_sent = False

    def send_serial(self, target_found, target_angle, target_distance):
         """
        send serial data after set intervals of time
        send angle of target and the distance needed to hit them, given in launcher's specs
        """

        #if there is no target, then loops over
        if not target_found:
            self.prev_time = time.time()
            pass
        else:
            #if enough time has passed, send an angle            
            if(time.time() - self.prev_time_angle > self.time_to_send_angle):
                self.ser_out.write("a = " + str(int(target_angle)))
                self.prev_time_angle = time.time()

            #if enough time has passed, send an angle and a distance to arm the launcher
            if (time.time() - self.prev_time) > self.time_to_arm:
                if(not self.arm_sent):
                    self.arm_sent = True
                    #self.ser_out.write("a = 20, power = 10")
                    self.ser_out.write("a= " + str(int(target_angle)) + ", power = " + str(int(self.distance_to_motor_power(target_distance))))
                    print ("a= " + str(target_angle) + ", power = " + str(int(self.distance_to_motor_power(target_distance))))
            #if enough time has passed, send a command to fire
            if(time.time() - self.prev_time) > self.time_to_shoot:
                # self.ser_out.write("fire")
                print ("fire")
                self.prev_time = time.time()
                self.arm_sent = False


    def distance_to_motor_power(self, distance):
        """
        performs a conversion to get distance into launcher specs
        """
        return .1565*(distance - 9.09)




class LidarView(object):
    """
    holds the processing for the LIDAR data
    """
    def __init__(self):
 
        self.angle_1 = 340
        self.angle_2 = 70
        self.target_angle = 0
        self.est_dist = 500
        self.est_threshold = 30
        self.r_min = 1000
        self.target_found = False


    def draw(self, dataArray, target_data):
        """
        computes the angle and distance of the target if there is one.
        """

        #if there is a target, then get the left angle, right angle, and other information
        if(len(target_data) > 0):
            self.angle_1 = target_data[0][0]
            self.angle_2 = target_data[0][1]
            self.target_angle = (self.angle_1 + self.angle_2)/2
            self.est_dist = target_data[0][2]
            self.target_found = True
        else:
            self.target_found = False


        self.r_min = 1000

        #assigns colors to all data points for classification, and also calculates the distance
        for i, obj in enumerate(dataArray):
            if obj != None:
                #makes angles nice
                if self.angle_1 < 0:
                    self.angle_1 = self.angle_1 + 360

                if self.angle_2 < 0:
                    self.angle_2 = self.angle_2 + 360

                #classifies data points
                if self.angle_1 > self.angle_2:

                    if (self.angle_2 <= obj[0]<= self.angle_1):
                        dot_color = pygame.Color('red')
                    else:
                        dot_color = pygame.Color('green')

                else:

                    if (self.angle_1 <= obj[0]<= self.angle_2):
                        dot_color = pygame.Color('green')
                    else:
                        dot_color = pygame.Color('red')

                if not self.target_found:
                    dot_color = pygame.Color('red')


                #calculates distance by finding the smallest distance in the angle range
                if (dot_color==pygame.Color('green')):
                    if obj[1]<self.r_min:
                        self.r_min = obj[1]

        return (self.target_found, -self.target_angle, self.r_min)



class TargetLocator(object):
    """
    holds the processing of the Kinect data to find targets
    """
    def __init__(self):
        self.people = []
        self.targets = []
        self.lidar_readings = {}
        self.target_readings = {}
        self.readings_list = []
        self.targeted = []
        self.threshold = 2
        self.kinectFOV = 57 #in degrees
        self.kinectHeight = 240.0
        self.kinectLength = 320.0

    def track(self, crowd):
        """
        uses the rectangles drawn around targets to find the angle range they lie in
        """
        if(len(crowd)> 0):
            self.people = []
            for (x, y, w, h) in crowd:
                #print (x, x+w)
                angleMax = (self.kinectLength/2 - x)/(self.kinectLength)*self.kinectFOV
                angleMin = (self.kinectLength/2 - (x+w))/(self.kinectLength)*self.kinectFOV
                self.people.append((angleMin, angleMax , self.get_distance_estimate(abs(y-h))))
            #print self.people
            return self.people
        else:
            return []


    def get_distance_estimate(self, height):
        return -.19012*height + 542.5
