import serial
import time

ser = serial.Serial(
	port = '/dev/ttyACM0', 
	baudrate = 115200,
	timeout = 0,
	writeTimeout = 0,
)
i = 0;
while True:
	if(i < 30):
		ser.write(str(i))
		print("angle = " + str(i))
	else:
		i = 0
	i += 5
	time.sleep(1)

ser.close()