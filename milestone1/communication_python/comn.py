import serial
import sys
import time
from threading import Thread

class Communication():

	def __init__(self, port='COM3', baudrate=115200, timeout=5):
		self.serial 	= serial.Serial(port=port,baudrate=baudrate,timeout=timeout)
		self.port 		= self.serial.getPort()
		self.baudrate 	= self.serial.getBaudrate()
		self.timeout 	= self.serial.getTimeout()

	def send(self, msg):
		# Send message to Arduino
		self.serial.write((str(msg)))

	def close(self):
		# Empty the buffer and close the connection
		self.serial.flush()
		self.serial.close()

if __name__ == '__main__':
	# Accept input from the user through the command line, for example: "python comn.py 1"
	comm = Communication()
	for arg in sys.argv:
		comm.send(arg)