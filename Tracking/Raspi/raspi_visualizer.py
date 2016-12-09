import serial
import sys
import math
import time
from threading import Thread
import cv2
import pygame


class RaspiVisuals(object):
	"""
	class whose sole purpose is to allow for visualizations for Raspi data, since Raspi isn't
	fast eonugh to do it itself. 

	***This file should be run on a separate computer, with a serial connection to the Raspi***
	"""
	def __init__(self):
		self.model = LidarModel()
		self.screen = pygame.display.set_mode(self.model.size)
		self.view = LidarView(self.screen, self.model)

		self.camera = KinectVisualizer()

		self.ser2 = serial.Serial()
		self.ser2.port = '/dev/ttyACM0'
		self.ser2.baudrate = 115200
		self.ser2.timeout = 1
		self.ser2.open()

		self.data = None
		self.target_data = []
  
	def visualize(self):
		self.data = self.ser2.read()
		if self.data[0] == 1:
			self.view.draw(self.data[1], self.target_data)
		else:
			self.target_data = self.camera.track(self.data[1])
			self.camera.render(self.data[1])

	def shut_down(self):
		cv2.destroyAllWindows()
		self.ser2_out.close()





class LidarView(object):
    """
    holds the visualization and processing for the LIDAR data
    """
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
        """
        draws our a projection map of the LIDAR's data, and also computes the angle and distance of the target
        if there is one.
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


        self.screen.fill(pygame.Color('grey'))
        self.r_min = 1000

        #assigns colors to all data points for visualizations, and also calculates the distance
        for i, obj in enumerate(dataArray):
            if obj != None:
                #makes angles nice
                if self.angle_1 < 0:
                    self.angle_1 = self.angle_1 + 360
                if self.angle_2 < 0:
                    self.angle_2 = self.angle_2 + 360
                #classifies datapoints
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


                #draws all the points
                x = int((obj[1]+3)*math.cos(obj[0]*math.pi/180)*self.scaling)
                y = int((obj[1]+3)*math.sin(obj[0]*math.pi/180)*self.scaling)
                pygame.draw.circle(self.screen, dot_color, (self.center[0] - x, self.model.height -(self.center[1] - y)), 2)

                #calculates distance by finding the closest point in the angle range of the target
                if (dot_color==pygame.Color('green')):
                    if obj[1]<self.r_min:
                        self.r_min = obj[1]

        #draws a radius to visualize distance of target from LIDAR                
        if self.target_found:
            if self.r_min >5:
                pygame.draw.circle(self.screen, pygame.Color('blue'), (self.center[0], self.model.height - (self.center[1])), int(self.r_min*self.scaling), 1)

                # if (obj[1] > (self.est_dist - self.est_threshold) and obj[1] < (self.est_dist + self.est_threshold) and dot_color==pygame.Color('green')):
                #     pygame.draw.circle(self.screen, pygame.Color('blue'), (self.center[0] - x, self.model.height -(self.center[1] - y)), 2)
                # else:
                #     pygame.draw.circle(self.screen, dot_color, (self.center[0] - x, self.model.height -(self.center[1] - y)), 2)
                #print (x,y)


        #draws angle lines to signify what zone the target is in
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
    """
    the model for the LIDAR visualization
    """
    def __init__(self):
        self.width = 1000
        self.height = 1000
        self.center = (self.width/2, self.height/2)
        self.size = (self.width, self.height)



class KinectVisualizer(object):
	"""
	the class that draws the frames from the Raspi
	"""
	def __init__(self):
		self.frame = None
		self.targets = None

		self.people = []
		self.kinectFOV = 57 #in degrees
		self.kinectHeight = 240.0
		self.kinectLength = 320.0


	def render(self, data):
		"""
		draws the rectangles onto the frame, and then shows it
		"""
		self.frame = data[0]
		self.targets = data[1]
		for (x,y,w,h) in self.targets:
			cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

		cv2.namedWindow('frame', 0)
		cv2.resizeWindow('frame', 320, 240)

		cv2.imshow('frame',self.frame)


	def track(self, data):
		"""
		uses the rectangles drawn around targets to find the angle range they lie in
		"""
		self.targets = data[1]
		if(len(self.targets)> 0):
			self.people = []
			for (x, y, w, h) in self.targets:
				angleMax = (self.kinectLength/2 - x)/(self.kinectLength)*self.kinectFOV
				angleMin = (self.kinectLength/2 - (x+w))/(self.kinectLength)*self.kinectFOV
				self.people.append((angleMin, angleMax , self.get_distance_estimate(abs(y-h))))
				#print self.people
			return self.people
		else:
			return []

	def get_distance_estimate(self, height):
		return -.19012*height + 542.5		



if __name__== "__main__":
	pictures = RaspiVisuals()

	while True:
		pictures.visualize()

	pictures.shut_down()

