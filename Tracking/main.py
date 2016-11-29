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

if __name__ == '__main__':
	pygame.init()
	model = lidar.LidarModel()
	screen = pygame.display.set_mode(model.size)
	view = lidar.LidarView(screen, model)

	while True:

		for event in pygame.event.get():
			if event.type == pygame.QUIT: sys.exit()
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

	ser.close()