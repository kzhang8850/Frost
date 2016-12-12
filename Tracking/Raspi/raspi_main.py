import multiprocessing
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
import raspi_body_detection
import raspi_lidar
import raspi_processor


class Frost(object):
    """
    the main class of Frost, our snowball launcher
    """
    def __init__(self):
        self.q = multiprocessing.Queue()

        self.ser = None
        self.ser_out = None
        self.ser2_out = None
        self.initialize_serial()

        self.supervisor = raspi_processor.Supervisor(self.ser_out)

        self.thread1 = raspi_body_detection.BodyThread(self.q)
        self.thread2 = raspi_lidar.LidarThread(self.q, self.ser)

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
        self.ser.write_timeout = 2     #timeout for write
        self.ser.open()

        #for self.serial output to arduino for launcher
        self.ser_out = serial.Serial()
        self.ser_out.port = '/dev/ttyACM1'
        self.ser_out.baudrate = 115200
        self.ser_out.timeout = 1
        self.ser_out.write_timeout = 0     #timeout for write
        self.ser_out.open()

        #for serial output to secondary computer for visualization
        # self.ser2_out = serial.Serial()
        # self.ser2_out.port='/dev/ttyACM0'
        # self.ser2_out.baudrate=115200
        # self.ser2_out.timeout = 1
        # self.ser2_out.writeTimeout = 0     #timeout for write
        # self.ser2_out.open()



    def run(self):
        """
        runs the multiprocessing for Frost's Kinect vision, LIDAR, and launching
        """

        while True:
            try:
                #if the queue has data, then take it and send it off to the approrpriate processing unit in supervisor
                if not self.q.empty():
                    self.xdata = self.q.get()
                    if self.xdata[0] == 1:
                        (self.target_found, self.target_angle, self.target_distance) = self.supervisor.view.draw(self.xdata[1], self.target_data)
                    else:
                        self.target_data = self.supervisor.targeter.track(self.xdata[1][1])
                    self.supervisor.serial_out.send_serial(self.target_found, self.target_angle, self.target_distance)

            except KeyboardInterrupt:
                print "keyboard"
                self.thread1.terminate()
                self.thread2.terminate()


                break


if __name__ == "__main__":
    frost = Frost()
    frost.run()
