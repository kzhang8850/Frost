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


class Frost(object):
    """
    the main class of Frost, our snowball launcher
    """
    def __init__(self):
        self.q = Queue.Queue()

        self.ser = None
        self.ser_out = None
        self.initialize_serial()

        self.supervisor = processor.Supervisor(self.ser_out)

        self.thread1_stop = threading.Event()
        self.thread1 = body_detection.BodyThread(q, thread1_stop)
        self.thread2_stop = threading.Event()
        self.thread2 = lidar.LidarThread(q, thread2_stop, self.ser)

        self.thread1.setDaemon = True
        self.thread2.setDaemon = True

        self.thread1.start()
        self.thread2.start()

        self.target_data = []
        self.target_angle = 0
        self.target_found = False
        self.target_distance = 0

        self.xdata = None



    def initialize_serial(self):
        """
        Serial stuff
        """

        #for serial input from arduino for LIDAR
        self.ser = serial.Serial()
        self.ser.port='/dev/ttyACM0'
        self.ser.baudrate=115200
        self.ser.timeout = 1
        self.ser.writeTimeout = 2     #timeout for write
        self.ser.open()

        #for self.serial output to arduino for launcher
        self.ser_out = serial.Serial()
        self.ser_out.port = '/dev/ttyACM1'
        self.ser_out.baudrate = 115200
        self.ser_out.timeout = 1
        self.ser_out.writeTimeout = 0     #timeout for write
        self.ser_out.open()



    def run(self):
        """
        runs the threading and the processes for Frost's Kinect vision, LIDAR, and launching
        """

        while True:
            try:
                #if the queue has data, then take it and send it off to the approrpriate processing unit in supervisor
                if not self.q.empty():
                    self.xdata = self.q.get()
                    if self.xdata[0] == 1:
                        (self.target_found, self.target_angle, self.target_distance) = self.supervisor.view.draw(self.xdata[1], self.target_data)
                    else:
                        self.target_data = self.supervisor.targeter.track(self.xdata[1])
                    self.supervisor.serial_out.send_serial(self.target_found, self.target_angle, self.target_distance)

                #shuwdown sequence for pygame    
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        sys._exit() 
                        break

            except KeyboardInterrupt:
                print "keyboard"
                thread1_stop.set()
                thread2_stop.set()


                break
        sys.exit()


if __name__ == "__main__":
    frost = Frost()
    frost.run()
