import serial
import sys
import time
from threading import Thread

class Communication():

	def __init__(self, port='COM3', baudrate=115200, timeout=5):
		self.serial 	= serial.Serial(port,baudrate,timeout)
		self.port 		= port
		self.baudrate 	= baudrate
		self.timeout 	= timeout

	def send(self, msg):
		# Send message to Arduino
		self.serial.write((str(msg) + '\r').encode())

	def read(self, timeout = 5):
		# Read message sent by the Arduino

		message = "No message"

		# Set up timeout
		timeout = time.time() + timeout

		while True:

			print timeout
			# Time out
			if time.time() > timeout:
				return message
				break

			message = self.serial.readline()
			time.sleep(1)

	def close(self):
		# Empty the buffer and close the connection
		self.serial.flush()
		self.serial.close()

if __name__ == '__main__':
	# Accept input from the user through the command line, for example: "python comn.py 1"
	comm = Communication()
	for arg in sys.argv:
		comm.send(arg)

	# thread = Thread(target = comm.read, args=[])
	# thread.start()
	# thread.join()