import serial
import sys
import time

class Communication():

	def __init__(self, port='COM3', baudrate=115200, timeout=5):
		# self.serial 	= serial.Serial(port,baudrate,timeout)
		self.port 		= port
		self.baudrate 	= baudrate
		self.timeout 	= timeout

	def send(self, msg):
		# Send message to Arduino
		self.serial.write(str(msg) + '\r'.encode())

	def read(self):
		# Read message sent by the Arduino
		return self.serial.read(10)

if __name__ == '__main__':
	# Accept input from the user through the command line, for example: "python comn.py 1"
	for arg in sys.argv:
		comm.send(arg)