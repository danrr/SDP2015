import serial
import sys
import time
from threading import Thread
import termios
import contextlib

class Communication():

	# REMEMBER TO CHANGE THE PORT FOR WINDOWS/MAC/LINUX
	def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=5):
		self.serial 	= serial.Serial(port,baudrate,timeout)
		self.port 		= port
		self.baudrate 	= baudrate
		self.timeout 	= timeout

	def send(self, msg):
		# Send message to Arduino
		self.serial.write(msg + 'd')

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

@contextlib.contextmanager
def raw_mode(file):
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)

def main():
	# Executing python comn.py will start the program (no arguments)
    print 'exit with ^C or ^D'	
    comm = Communication()

    with raw_mode(sys.stdin):
        try:
            while True:
                ch = sys.stdin.read(1)
                if not ch or ch == chr(4):
                    break
                char = str.upper(ch)
                print char
                comm.send(char)
        except (KeyboardInterrupt, EOFError):
            pass

if __name__ == '__main__':
    main()
