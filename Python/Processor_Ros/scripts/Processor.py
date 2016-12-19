#!/usr/bin/env python
#################################################################################################################################
"""
Processor Module - Acts as Frost's Cognitive Thinking

Contains Four Classes that are utilized by the main module to calculate and perform actions on incoming data
1. SerialOut - pushes data to the Arduino on Frost's launcher
2. LidarView - Computes and Visualizes data from the LIDAR module
3. TargetLocator - Computes where people's coordinates for the launcher from the Kinect module

Written by Kevin Zhang, Cedric Kim, and Jeremy Garcia
"""
################################################################################################################################

import time
import serial
import pygame
import sys
import math
import cv2
import numpy as np
import os
import rospy
from frost_lidar.msg import Polar_Array


def distance_to_motor_power(self, distance):
    """
    performs a conversion to get distance into launcher specs
    """
    return .0897*(distance - 3.7082)


class LidarView(object):
    """
    holds the visualization and processing for the LIDAR data
    """
    def __init__(self, screen, model):
        rospy.init_node('Processor')
        self.sub = rospy.Subscriber('/lidar_data', Polar_Array, self.laser_callback)
        #self.sub = rospy.Subscriber('/imu/heading', Float32, self.imu_callback)
        #self.pub = rospy.Publisher('/wpt/cmd_vel', Float32MultiArray, queue_size = 1)
        
        self.screen = screen
        self.model = model
        self.center = model.center
        self.scaling = .3
        self.angle_1 = 340
        self.angle_2 = 70
        self.target_angle = 0
        self.est_dist = 500
        #self.est_threshold = 30
        self.r_min = 1000
        self.target_found = False
        self.data_wedge = []
        self.left_angle_min = 500
        self.right_angle_max = -500
        self.distance_threshold = 20
        self.dataArray = []

    def laser_callback(self, scan):
        ##data array is a list of polar msgs, so self.dataArray[i].theta = angle,
        ## and self.dataArray[i].r = distance
        self.dataArray = scan.Polar_Array
    def draw(self):
        """
        draws our a projection map of the LIDAR's data, and also computes the angle and distance of the target
        if there is one.
        """
        
        #if there is a target, then get the left angle, right angle, and other information
        """if(len(target_data) > 0):
            self.angle_1 = target_data[0][0] #left angle of first target
            self.angle_2 = target_data[0][1] #right angle of first target

            self.est_dist = target_data[0][2]
            self.target_found = True
        else:
            self.target_found = False
            self.angle_1 = 0
            self.angle_2 = 0"""
            #self.target_angle = 0

        self.screen.fill(pygame.Color('grey'))
        self.r_min = 1000
        #assigns colors to all data points for visualizations, and also calculates the distance
        # 3 sets of data wedge
        self.data_wedge = []
        for i, obj in enumerate(self.dataArray):
            if obj != None:
                angle = obj.theta
                distance = obj.r
                if angle>180:
                    angle = angle-360
                if angle>self.angle_1 and angle<self.angle_2:
                    dot_color = pygame.Color('green')
                    self.data_wedge.append([angle, obj[1]])
                else:
                    dot_color = pygame.Color('red')

                x = int((distance+3)*math.cos(angle*math.pi/180)*self.scaling)
                y = int((distance+3)*math.sin(angle*math.pi/180)*self.scaling)
                pygame.draw.circle(self.screen, dot_color, (self.center[0] - x, self.model.height -(self.center[1] - y)), 2)

        #calculates distance by finding the closest point in the angle range of the target

        if(len(self.data_wedge)> 0):
            #sort data wedge by distance
            self.data_wedge.sort(key=lambda x: int(x[1]))
            #get first data_wedge distance
            if(self.data_wedge[0][1] > 10):
                self.r_min = self.data_wedge[0][1]
            else:
                self.r_min = self.data_wedge[1][1]
            self.left_angle_min = 500
            self.right_angle_max = -500
            for i, obj in enumerate(self.data_wedge):
                angle = obj[0]
                if (abs(obj[1] - self.r_min) < self.distance_threshold):
                    if(angle < self.left_angle_min):
                        self.left_angle_min = angle
                    if(angle > self.right_angle_max):
                        self.right_angle_max = angle

         #draws a radius to visualize distance of target from LIDAR                
        if self.target_found:
            if self.r_min >10:
                pygame.draw.circle(self.screen, pygame.Color('blue'), (self.center[0], self.model.height - (self.center[1])), int(self.r_min*self.scaling), 1)
        self.target_angle = ((self.right_angle_max + self.left_angle_min)/2)*.8
        #draws angle lines to signify what zone the target is in
        line1_x_pos = int(1000*math.cos(self.angle_1*math.pi/180))
        line1_y_pos = int(1000*math.sin(self.angle_1*math.pi/180))
        line2_x_pos = int(1000*math.cos(self.angle_2*math.pi/180))
        line2_y_pos = int(1000*math.sin(self.angle_2*math.pi/180))
        line_target_x_pos = int(1000*math.cos(self.target_angle*math.pi/180))
        line_target_y_pos = int(1000*math.sin(self.target_angle*math.pi/180))
        pygame.draw.circle(self.screen, pygame.Color('yellow'), (self.center[0], self.model.height - (self.center[1])), 3)
        if self.target_found:
            pygame.draw.line(self.screen, pygame.Color('green'), (self.center[0], self.model.height - (self.center[1])),(self.center[0]-line1_x_pos, self.model.height - (self.center[1]-line1_y_pos)),1)
            pygame.draw.line(self.screen, pygame.Color('green'), (self.center[0], self.model.height - (self.center[1])),(self.center[0]-line2_x_pos, self.model.height - (self.center[1]-line2_y_pos)),1)
            pygame.draw.line(self.screen, pygame.Color('black'), (self.center[0], self.model.height - (self.center[1])), (self.center[0] - line_target_x_pos, self.model.height - (self.center[1] - line_target_y_pos)), 1)

        pygame.display.update()
        if(self.r_min == 1000):
            self.target_found = False
        return (self.target_found, -self.target_angle, self.r_min)

class LidarModel(object):
    """
    the model for the LIDAR visualization
    """
    def __init__(self):
        self.width = 1000
        self.height = 1000
        self.center = (self.width/2, self.height/2)
        self.size = (self.width, self.height)


class TargetLocator(object):
    """
    holds the processing of the Kinect data to find targets
    """
    def __init__(self):
        self.people = []
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
                angleMax = (self.kinectLength/2 - x)/(self.kinectLength)*self.kinectFOV
                angleMin = (self.kinectLength/2 - (x+w))/(self.kinectLength)*self.kinectFOV
                self.people.append((angleMin, angleMax , self.get_distance_estimate(abs(y-h))))
            return self.people
        else:
            return []


    def get_distance_estimate(self, height):
        return -.19012*height + 542.5



if __name__ == '__main__':
    print 'start'
    model = LidarModel();
    screen = pygame.display.set_mode(model.size)
    view = LidarView(screen, model)
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        view.draw()
        r.sleep()
