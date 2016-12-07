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
    def __init__(self, ser_out):
        self.model = LidarModel()
        self.screen = pygame.display.set_mode(self.model.size)
        self.view = LidarView(self.screen, self.model)
        self.serial_out = SerialOut(ser_out)
        self.targeter = TargetLocator()

class SerialOut(object):
    def __init__(self, ser_out):
        self.ser_out = ser_out
        self.prev_time = time.time()
        self.time_to_arm = 6
        self.time_to_shoot = 8
        self.time_to_send_angle = .5
        self.prev_time_angle = time.time()
        self.arm_sent = False
        #self.fire_sent = False

    def send_serial(self, target_found, target_angle, target_distance):
        if not target_found:
            self.prev_time = time.time()
        else:
            if(time.time() - self.prev_time_angle > self.time_to_send_angle):
                self.ser_out.write("a = " + str(int(target_angle*1.15)))
                self.prev_time_angle = time.time()
        if (time.time() - self.prev_time) > self.time_to_arm:
            if(not self.arm_sent):
                self.arm_sent = True
                #self.ser_out.write("a = 20, power = 10")
                self.ser_out.write("a= " + str(int(target_angle)) + ", power = " + str(int(self.distance_to_motor_power(target_distance))))
                #self.ser_out.write("a= " + str(target_angle) + ", power = " + str(int(self.distance_to_motor_power(target_distance))))
                print ("a= " + str(target_angle) + ", power = " + str(int(self.distance_to_motor_power(target_distance))))
                self.ser_out.write(".")
        if(time.time() - self.prev_time) > self.time_to_shoot:
            #self.ser_out.write(".")
            #self.ser_out.write("fire")
            print ("fire")
            self.prev_time = time.time()
            self.arm_sent = False
    def distance_to_motor_power(self, distance):
        return .1565*(distance - 9.09)




class LidarView(object):
    def __init__(self, screen, model):
        self.screen = screen
        self.model = model
        self.center = model.center
        self.scaling = .3

        self.angle_1 = 340
        self.angle_2 = 70
        self.target_angle = 0
        self.est_dist = 500
        self.est_threshold = 30
        self.r_min = 1000
        self.target_found = False
    def draw(self, dataArray, target_data):
        if(len(target_data) > 0):
            self.angle_1 = target_data[0][0]
            self.angle_2 = target_data[0][1]
            self.target_angle = (self.angle_1 + self.angle_2)/2
            self.est_dist = target_data[0][2]
            self.target_found = True
        else:
            self.target_found = False




        self.screen.fill(pygame.Color('grey'))
        self.r_min = 1000
        for i, obj in enumerate(dataArray):
            if obj != None:
                if self.angle_1 < 0:
                    self.angle_1 = self.angle_1 + 360
                if self.angle_2 < 0:
                    self.angle_2 = self.angle_2 + 360
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
                x = int((obj[1]+3)*math.cos(obj[0]*math.pi/180)*self.scaling)
                y = int((obj[1]+3)*math.sin(obj[0]*math.pi/180)*self.scaling)
                pygame.draw.circle(self.screen, dot_color, (self.center[0] - x, self.model.height -(self.center[1] - y)), 2)
                if (dot_color==pygame.Color('green')):
                    if obj[1]<self.r_min:
                        self.r_min = obj[1]
        if self.target_found:
            if self.r_min >5:
                pygame.draw.circle(self.screen, pygame.Color('blue'), (self.center[0], self.model.height - (self.center[1])), int(self.r_min*self.scaling), 1)

                # if (obj[1] > (self.est_dist - self.est_threshold) and obj[1] < (self.est_dist + self.est_threshold) and dot_color==pygame.Color('green')):
                #     pygame.draw.circle(self.screen, pygame.Color('blue'), (self.center[0] - x, self.model.height -(self.center[1] - y)), 2)
                # else:
                #     pygame.draw.circle(self.screen, dot_color, (self.center[0] - x, self.model.height -(self.center[1] - y)), 2)
                #print (x,y)

        line1_x_pos = int(1000*math.cos(self.angle_1*math.pi/180))
        line1_y_pos = int(1000*math.sin(self.angle_1*math.pi/180))
        line2_x_pos = int(1000*math.cos(self.angle_2*math.pi/180))
        line2_y_pos = int(1000*math.sin(self.angle_2*math.pi/180))

        pygame.draw.circle(self.screen, pygame.Color('yellow'), (self.center[0], self.model.height - (self.center[1])), 3)
        if self.target_found:
            pygame.draw.line(self.screen, pygame.Color('green'), (self.center[0], self.model.height - (self.center[1])),(self.center[0]-line1_x_pos, self.model.height - (self.center[1]-line1_y_pos)),1)
            pygame.draw.line(self.screen, pygame.Color('green'), (self.center[0], self.model.height - (self.center[1])),(self.center[0]-line2_x_pos, self.model.height - (self.center[1]-line2_y_pos)),1)

        # if self.angle_1>self.angle_2:
        #     pygame.draw.arc(self.screen, pygame.Color('blue'), (self.center[0]-((self.est_dist+self.est_threshold)*self.scaling), self.model.height - (self.center[1])-((self.est_dist+self.est_threshold)*self.scaling),(self.est_dist+self.est_threshold)*2*self.scaling,(self.est_dist+self.est_threshold)*2*self.scaling),(self.angle_1-180)*(math.pi/180.),(self.angle_2+180)*(math.pi/180))
        #     pygame.draw.arc(self.screen, pygame.Color('blue'), (self.center[0]-((self.est_dist-self.est_threshold)*self.scaling), self.model.height - (self.center[1])-((self.est_dist-self.est_threshold)*self.scaling),(self.est_dist-self.est_threshold)*2*self.scaling,(self.est_dist-self.est_threshold)*2*self.scaling),(self.angle_1-180)*(math.pi/180.),(self.angle_2+180)*(math.pi/180))
        # else:
        #     pygame.draw.arc(self.screen, pygame.Color('blue'), (self.center[0]-((self.est_dist+self.est_threshold)*self.scaling), self.model.height - (self.center[1])-((self.est_dist+self.est_threshold)*self.scaling),(self.est_dist+self.est_threshold)*2*self.scaling,(self.est_dist+self.est_threshold)*2*self.scaling),(self.angle_1+180)*(math.pi/180.),(self.angle_2+180)*(math.pi/180))
        #     pygame.draw.arc(self.screen, pygame.Color('blue'), (self.center[0]-((self.est_dist-self.est_threshold)*self.scaling), self.model.height - (self.center[1])-((self.est_dist-self.est_threshold)*self.scaling),(self.est_dist-self.est_threshold)*2*self.scaling,(self.est_dist-self.est_threshold)*2*self.scaling),(self.angle_1+180)*(math.pi/180.),(self.angle_2+180)*(math.pi/180))

        pygame.display.update()
        return (self.target_found, -self.target_angle, self.r_min)


class LidarModel(object):
    def __init__(self):
        self.width = 1000
        self.height = 1000
        self.center = (self.width/2, self.height/2)
        self.size = (self.width, self.height)





class TargetLocator(object):
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

		# for person in self.people:
		# 	print person

		# self.targets = self.find_targets()
		# print self.targets

    def get_distance_estimate(self, height):
        return -.19012*height + 542.5

	def find_targets(self):

		while len(self.lidar_readings) < 360:
			angle, distance = get_lidar_reading()
			self.lidar_readings[angle] = distance
		for person in self.people:
			for ang, dist in self.lidar_readings.items():
				if ang > person[0] and ang < person[1]:
					self.target_readings[ang] = dist
			self.readings_list = self.target_readings.items()
			self.readings_list = sorted(self.readings_list, key=lambda tup: tup[0])
			count = -1
			comparecount = 0
			target = None
			comparison = self.readings_list[0]
			for angle, distance in self.readings_list:
				if abs(distance - comparison[1]) < 2:
					count += 1
				else:
					if count > comparecount:
						comparecount = count
						target = comparison
					comparison = angle, distance
					count = 0

			self.targeted.append(target[0] + count/2, target[1])

		return self.targeted
