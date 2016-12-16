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


class Supervisor(object):
    """
    main class - instantiates all processing classes for LIDAR, Kinect, and SerialOut
    """
    def __init__(self, ser_out):
        self.model = LidarModel()
        self.screen = pygame.display.set_mode(self.model.size)
        self.view = LidarView(self.screen, self.model)
        self.serial_out = SerialOut(ser_out)
        self.targeter = TargetLocator()


class SerialOut(object):
    """
    class that holds processing to push output data to the launcher for panning and shooting a set distance
    """
    def __init__(self, ser_out):
        self.ser_out = ser_out
        self.prev_time = time.time()
        self.time_to_arm = 6
        self.time_to_shoot = 8
        self.time_to_send_angle = .5
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
                self.ser_out.write("a = " + str(int(target_angle*1.15)))
                self.prev_time_angle = time.time()

            #if enough time has passed, send an angle and a distance to arm the launcher
            if (time.time() - self.prev_time) > self.time_to_arm:
                if(not self.arm_sent):
                    self.arm_sent = True
                    self.ser_out.write("a= " + str(int(target_angle)) + ", power = " + str(int(self.distance_to_motor_power(target_distance))))
                    print ("a= " + str(target_angle) + ", power = " + str(int(self.distance_to_motor_power(target_distance))))
                    self.ser_out.write(".")

            #if enough time has passed, send a command to fire
            if(time.time() - self.prev_time) > self.time_to_shoot:
                # self.ser_out.write(".")
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
    holds the visualization and processing for the LIDAR data
    """
    def __init__(self, screen, model):
        self.screen = screen
        self.model = model
        self.center = model.center
        self.scaling = .3


        #starting angle ranges
        self.angle_1 = 340
        self.angle_2 = 70

        #angle of the target
        self.target_angle = 0

        #whether the target has been found
        self.target_found = False

        #datapoints corresponding to the person, in tuples of angle and distance
        self.person = []

        #the person' average distance away from Frost
        self.person_distance = 5000

        #a short sliding window to check for stray datapoints
        self.history = []
        self.started = False


    def draw(self, dataArray, target_data):
        """
        draws our a projection map of the LIDAR's data, and also computes the angle and distance of the target
        if there is one.
        """

        #sorts the data array, using default to sort by angle, which is the first index
        dataArray = sorted(dataArray)

        #if there is a target, then get the left angle, right angle, and other information
        if(len(target_data) > 0):
            self.angle_1 = target_data[0][0]
            self.angle_2 = target_data[0][1]
            self.target_found = True

            #makes angles nice
            if self.angle_1 < 0:
                self.angle_1 += 360
            if self.angle_2 < 0:
                self.angle_2 += 360
            if self.angle_1 > self.angle_2:
                self.angle_1 -= 360
        else:
            self.target_found = False

        self.screen.fill(pygame.Color('grey'))
        self.person_distance = 5000

        #assigns colors to all data points for visualizations, and also calculates the distance
        for i in range(len(dataArray)):
            if dataArray[i] is not None:
                
                angle = dataArray[i][0]
                distance = dataArray[i][1]

                #classifies datapoints
                if (self.angle_1 <= angle <= self.angle_2):
                    dot_color = pygame.Color('green')
                else:
                    dot_color = pygame.Color('red')

                #catches all no targets found to ensure red
                if not self.target_found:
                    dot_color = pygame.Color('red')
                
                #draws all the points
                x = int((distance+3)*math.cos(angle*math.pi/180)*self.scaling)
                y = int((distance+3)*math.sin(angle*math.pi/180)*self.scaling)

                pygame.draw.circle(self.screen, dot_color, (self.center[0] - x, self.model.height -(self.center[1] - y)), 2)

                #calculates distance by finding the closest point in the angle range of the target
                if (dot_color==pygame.Color('green')):
                    if abs(self.person_distance - distance) <= 3:
                        self.person_distance *= len(self.person)
                        self.person.append((angle, distance))
                        self.person_distance += distance
                        self.person_distance /= len(self.person)
                    else:
                        if self.started:
                            if (abs(self.person_distance - distance) > 3) and (distance < self.person_distance) and (abs(np.mean(self.history) - distance) < 15):
                                self.person = []
                                self.person.append((angle, distance))
                                self.person_distance = distance
                        else:
                            if (abs(self.person_distance - distance) > 3) and (distance < self.person_distance):
                                self.person = []
                                self.person.append((angle, distance))
                                self.person_distance = distance


        #find person angle
        self.target_angle = (self.person[0][0] + self.person[-1][0])/2

        print self.person
        #acknowledge first loop has passed
        if not self.started:
            self.started = True

        #update history
        if len(self.history) < 3:
            self.history.append(self.person_distance)
        else:
            self.history.pop(0)
            self.history.append(self.person_distance)

        #draws a radius to visualize distance of target from LIDAR                
        if self.target_found and (self.person_distance > 5):
                pygame.draw.circle(self.screen, pygame.Color('blue'), (self.center[0], self.model.height - (self.center[1])), int(self.person_distance*self.scaling), 1)

        #draws angle lines to signify what zone the target is in
        line1_x_pos = int(1000*math.cos(self.angle_1*math.pi/180))
        line1_y_pos = int(1000*math.sin(self.angle_1*math.pi/180))
        line2_x_pos = int(1000*math.cos(self.angle_2*math.pi/180))
        line2_y_pos = int(1000*math.sin(self.angle_2*math.pi/180))

        pygame.draw.circle(self.screen, pygame.Color('yellow'), (self.center[0], self.model.height - (self.center[1])), 3)

        if self.target_found:
            pygame.draw.line(self.screen, pygame.Color('green'), (self.center[0], self.model.height - (self.center[1])),(self.center[0]-line1_x_pos, self.model.height - (self.center[1]-line1_y_pos)),1)
            pygame.draw.line(self.screen, pygame.Color('green'), (self.center[0], self.model.height - (self.center[1])),(self.center[0]-line2_x_pos, self.model.height - (self.center[1]-line2_y_pos)),1)

        pygame.display.update()
        return (self.target_found, -self.target_angle, self.person_distance)


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



if __name__ == "__main__":
    #testing stuff
    test = Supervisor(None)
    testdataarray = [[1,40],[2,45],[3,44],[4,46],[5,47],[6,45],[7,44],[8,45],[9,45],[10,46],[11,47],[12,45],[13,45],[14,45],[15,46],[16,45],[17,24],[18,24],[19,25],[20,26],[21,24],[22,46],[23,46],[24,45],[25,45],[26,45],
                    [27,45],[28,44],[29,46],[30,45]]
    testangles = [(10, 25), (45,45)]
    print test.view.draw(testdataarray, testangles)
    testdataarray = [[1,40],[2,45],[3,44],[4,46],[5,47],[6,45],[7,44],[8,45],[9,45],[10,46],[11,47],[12,45],[13,45],[14,45],[15,46],[16,45],[17,24],[18,24],[19,25],[20,6],[21,24],[22,46],[23,46],[24,45],[25,45],[26,45],
                    [27,45],[28,44],[29,46],[30,45]]
    testangles = [(10, 25), (45,45)]
    print test.view.draw(testdataarray, testangles)
    testdataarray = [[1,40],[2,45],[3,44],[4,46],[5,47],[6,45],[7,44],[8,45],[9,45],[10,46],[11,47],[12,45],[13,45],[14,45],[15,46],[16,45],[17,24],[18,24],[19,25],[20,46],[21,24],[22,46],[23,46],[24,45],[25,45],[26,45],
                    [27,45],[28,44],[29,46],[30,45]]
    testangles = [(10, 25), (45,45)]
    print test.view.draw(testdataarray, testangles)


