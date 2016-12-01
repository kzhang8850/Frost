import threading
import Queue
import os
import TestLidar
import TestBodyDetection
import ThreadingTest
import time
import serial
import binascii
import pygame
import sys
import math
import cv2
import numpy as np
import threadtest1
import threadtest2

q = Queue.Queue()

thread1_stop = threading.Event()
thread1 = ThreadingTest.Chicken(q, thread1_stop)
thread2_stop = threading.Event()
thread2 = threadtest2.Rooster(q, thread2_stop)

thread1.setDaemon = True
thread2.setDaemon = True

thread1.start()
thread2.start()




# #Targeter = TestBodyDetection.TargetLocator()
# model = TestLidar.LidarModel()
# screen = pygame.display.set_mode(model.size)
# view = TestLidar.LidarView(screen, model)
# #bodies = TestBodyDetection.BodyDetector()
#
# #thread1_stop = threading.Event()
# #thread1 = TestBodyDetection.BodyDetectorThread(q, thread1_stop, bodies)
# thread3_stop = threading.Event()
# thread3 = ThreadingTest.TestThread(q, thread3_stop, model, screen, view)
# thread2_stop = threading.Event()
# thread2 = TestLidar.LidarThread(q, thread2_stop, model, screen, view)
#
#
# #thread1.setDaemon = True
# thread3.setDaemon = True
# thread2.setDaemon = True
#
# #thread1.start()
# thread3.start()
# thread2.start()

# while True:
# 	for event in pygame.event.get():
# 	    if event.type == pygame.QUIT:
# 	        thread2_stop.set()
# 	        thread3_stop.set()
# 	        sys.exit()
while True:
    try:
		if not q.empty():
			# print 'not empty'
			xdata = q.get()
			# print xdata

		# for event in pygame.event.get():
		# 	if event.type == pygame.QUIT:
		# 		thread2_stop.set()
		# 		sys.exit()

    except KeyboardInterrupt:
        print "keyboard"
        #thread1_stop.set()
        thread1_stop.set()
        thread2_stop.set()

        break
	# k = cv2.waitKey(30) & 0xff
	# if k == 27:
	# 	break
os._exit()
