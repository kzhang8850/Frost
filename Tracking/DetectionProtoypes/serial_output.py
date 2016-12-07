import serial
import binascii
import sys
import math
import time
from threading import Thread



class SerialThread(Thread):
    def __init__(self, queue, stop, ser_out):
        super(SerialThread, self).__init__()
        self.queue = queue
        self.stop_event = stop
        self.serial = SerialOut(ser_out)

    def run(self):
        while not self.stop_event.is_set():
            print self.queue.get()
            #self.serial.send_serial(self.queue.get())

        self.ser_out.close()


class SerialOut(object):
    def __init__(self, ser_out):
        self.ser_out = ser_out
        self.prev_time = time.time()
        self.time_to_arm = 3
        self.time_to_shoot = 6
        self.time_to_send_angle = .5
        self.prev_time_angle = time.time()
        self.arm_sent = False
        #self.fire_sent = False

    def send_serial(self, target_found, target_angle, target_distance):
        if not target_found:
            self.prev_time = time.time()
        else:
            if(time.time() - self.prev_time_angle > self.time_to_send_angle):
                self.ser_out.write("a = " + str(int(target_angle)))
                self.prev_time_angle = time.time()
        if (time.time() - self.prev_time) > self.time_to_arm:
            if(not self.arm_sent):
                self.arm_sent = True
                #self.ser_out.write("a = 20, power = 10")
                self.ser_out.write("a= " + str(int(target_angle)) + ", power = " + str(int(self.distance_to_motor_power(target_distance))))
                #self.ser_out.write("a= " + str(target_angle) + ", power = " + str(int(self.distance_to_motor_power(target_distance))))
                print ("a= " + str(target_angle) + ", power = " + str(int(self.distance_to_motor_power(target_distance))))
                self.ser_out.write(" . ")
        if(time.time() - self.prev_time) > self.time_to_shoot:
            #self.ser_out.write(".")
            #self.ser_out.write("fire")
            print ("fire")
            self.prev_time = time.time()
            self.arm_sent = False
    def distance_to_motor_power(self, distance):
        return .1565*(distance - 9.09)
