import threading
import Queue
import os
import time
import serial
import binascii
import pygame
import sys
import math
import cv2
import numpy as np
import body_detection
import lidar
import processor



#####Serial Stuff
ser = serial.Serial()
ser.port='/dev/ttyACM0'
ser.baudrate=115200
ser.parity=serial.PARITY_NONE

ser.timeout = 1
ser.xonxoff = False     #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
ser.writeTimeout = 2     #timeout for write
ser.open()

q = Queue.Queue()


supervisor = processor.Supervisor()

thread1_stop = threading.Event()
thread1 = body_detection.BodyThread(q, thread1_stop)

thread2_stop = threading.Event()
thread2 = lidar.LidarThread(q, thread2_stop, ser)

thread1.setDaemon = True
thread2.setDaemon = True

thread1.start()
thread2.start()
target_data = []

while True:
    try:
        if not q.empty():
            xdata = q.get()
            if xdata[0] == 1:
                supervisor.view.draw(xdata[1], target_data)
            else:
                target_data = supervisor.targeter.track(xdata[1])
                #print target_data
                #print xdata[1]
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                os._exit() 
                break

    except KeyboardInterrupt:
        print "keyboard"
        thread1_stop.set()
        thread2_stop.set()

        break
os._exit()
