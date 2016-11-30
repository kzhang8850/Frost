import threading
import lidar
import body_detection
import time
import serial
import binascii
import pygame
import sys
import math
import cv2
import numpy as np

angle_1 = 340;
angle_2 = 70;
est_dist = 500;
est_threshold = 30;
class SerialInput(object):
	def __init__(self, view):
		self.view = view
		self.ser = serial.Serial()
		self.ser.port='/dev/ttyACM0'
		self.ser.baudrate=115200
		self.ser.parity=serial.PARITY_NONE
		#ser.stopbits=serial.STOPBITS_ONE,
		#bytesize=serial.SEVENBITS
		self.ser.timeout = 1
		self.ser.xonxoff = False     #disable software flow control
		self.ser.rtscts = False     #disable hardware (RTS/CTS) flow control
		self.ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
		self.ser.writeTimeout = 2     #timeout for write
		self.ser.open()

		self.inSync = False
		self.sync = 0
		self.bytesRead = 0
		self.j= 0
		self.counter = 0
		self.dataReady = False
		self.dataArray = []
		self.tempAngle = 0
		self.tempDistance = 0
		self.angle = 0
		self.distance = 0
	def readSerial(self):
		bytesRead = self.ser.inWaiting()
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
					#print("HighBit:")
					#print(ord(data)<<8)
					self.tempAngle = ord(data) << 8
					self.j += 1
					self.dataReady = False
				elif(self.j == 1):
					#print("LowBit:")
					#print(ord(data))
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
					self.dataArray.append((self.angle, self.distance))
					#print(angle , distance)
					#time.sleep()

			if (self.sync == 4):
				self.inSync = True
				self.counter += 1
				self.view.draw(self.dataArray)
				self.dataArray = []

def camera_input():
	crowd = Bodies.find_bodies()


if __name__ == '__main__':
	#Bodies = body_detection.BodyDetector()
	
	model = lidar.LidarModel()
	screen = pygame.display.set_mode(model.size)
	view = lidar.LidarView(screen, model)
	serial = SerialInput(view)
	while True:

		for event in pygame.event.get():
			if event.type == pygame.QUIT: sys.exit()

		serial.readSerial()
		#t = threading.Thread(target = serial.readSerial())
		#t.start()
		#w = threading.Thread(target = camera_input)

		#t.start()
		#w.start()

		#k = cv2.waitKey(30) & 0xff

		#if k == 27:
		#	break

	ser.close()
	Bodies.cam.release()
	cv2.destroyAllWindows()