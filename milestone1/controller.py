import argparse
import time
from communication_python.comn import Communication

if __name__ == '__main__':

	# Accept arguments for task and connection when running the controller
	parser = argparse.ArgumentParser()

	parser.add_argument("--task", help="1 = Forward 10cm, 2 = Forward 50cm, 3 = Backward, 4 = Kick", default=1)
	parser.add_argument("--port", help="E.g.: COM3", default="COM3")
	parser.add_argument("--baudrate", help="E.g.: 115200", default=115200)
	parser.add_argument("--timeout", help="Expressed in seconds, E.g.: 5", default=5)

	args = parser.parse_args()

	# Set up connection between the PC and the Arduino
	comm = Communication(port=args.port,baudrate=args.baudrate,timeout=args.timeout)

	print "Port:  " + str(comm.port) + ", Baudrate: " + str(comm.baudrate) + ", Timeout: " + str(comm.timeout)

	# Milestone 1 Tasks

	def forward10():
		# Start moving forward 10cm
		comm.send("Ad")
		print "Message: Move Forward"
		time.sleep(0.42)
		comm.send("Cd")
		print "Message: Stop"

	def forward50():
		# Start moving forward 50cm
		comm.send("Ad")
		print "Message: Move Forward"
		time.sleep(2.50)
		comm.send("Cd")
		print "Message: Stop"

	def backward():
		# Start moving backwards
		comm.send("Bd")
		print "Message: Move Backward"
		time.sleep(0.9)
		comm.send("Cd")
		print "Message: Stop"

	def kickUp():
		# Start kicking once uphill
		comm.send("D2")
		print "Message: Kick up pls"

	def kickDown():
		# Start kicking once downhill
		comm.send("D ")
		print "Message: Kick down pls"


	tasks = {
				1 : forward10,
				2 : forward50,
				3 : backward,
				4 : kickUp,
				5 : kickDown,
	}

	# Execute given task
	tasks[int(args.task)]()

	# End
	comm.close()