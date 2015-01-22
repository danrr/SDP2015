import serial
import sys

class Communication():

	def __init__(self, port='COM3', baudrate=115200, timeout=5):
		self.serial = serial.Serial(port,baudrate,timeout)

	def send(self, msg):
		# Send message to Arduino
		self.serial.write(str(msg) + '\r'.encode())

	def read(self):
		# Read message sent by the Arduino
		return self.serial.read(10)

comm = Communication()
# Accept input from the user through the command line, for example: "python comn.py 1"
for arg in sys.argv:
    comm.send(arg)
#print comm.read()

