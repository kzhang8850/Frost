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
        self.serial = Serial(ser_out)

    def run(self):
        while not self.stop_event.is_set():
            serial.SendSerial("a = 20, power = 20")
            serial.SendSerial("fire")


        self.ser_out.close()

class Serial(object):
    def __init__(self, ser_out):
        self.ser_out = ser_out

    def SendSerial(self, msg):
        self.ser_out.write(msg)
        self.ser_out.write(msg)

