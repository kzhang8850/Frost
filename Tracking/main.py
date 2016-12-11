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
ser.port='/dev/ttyACM1'
ser.baudrate=115200
ser.parity=serial.PARITY_NONE

ser.timeout = 1
ser.xonxoff = False     #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
ser.writeTimeout = 2     #timeout for write
ser.open()

##for serial output to arduino
ser_out = serial.Serial()
ser_out.port='/dev/ttyACM0'
ser_out.baudrate=115200
ser_out.parity=serial.PARITY_NONE

ser_out.timeout = 1
ser_out.xonxoff = False     #disable software flow control
ser_out.rtscts = False     #disable hardware (RTS/CTS) flow control
ser_out.dsrdtr = False       #disable hardware (DSR/DTR) flow control
ser_out.writeTimeout = 0     #timeout for write
ser_out.open()



q = Queue.Queue()


supervisor = processor.Supervisor(ser_out)

thread1_stop = threading.Event()
thread1 = body_detection.BodyThread(q, thread1_stop)

thread2_stop = threading.Event()
thread2 = lidar.LidarThread(q, thread2_stop, ser)

#thread3_stop = threading.Event()
#thread3 = serial_output.SerialThread(q, thread3_stop, ser_out)

thread1.setDaemon = True
thread2.setDaemon = True
#thread3.setDaemon = True

#thread1.start()
thread2.start()
#thread3.start()

target_data = []
target_angle = 0
target_found = False
target_distance = 0

while True:
    try:
        if not q.empty():
            xdata = q.get()
            if xdata[0] == 1:
                (target_found, target_angle, target_distance) = supervisor.view.draw(xdata[1], target_data)
            else:
                target_data = supervisor.targeter.track(xdata[1])
            supervisor.serial_out.send_serial(target_found, target_angle, target_distance)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys._exit() 
                break

    except KeyboardInterrupt:
        print "keyboard"
        thread1_stop.set()
        thread2_stop.set()
        #thread3_stop.set()

        break
sys._exit()
