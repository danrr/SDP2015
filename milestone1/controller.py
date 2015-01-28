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
	communication = Communication(port=args.port,baudrate=args.baudrate,timeout=args.timeout)

	print "Port:  " + str(communication.port) + ", Baudrate: " + str(communication.baudrate) + ", Timeout: " + str(communication.timeout)

	# Milestone 1 Tasks

	def forward10(comm):
		# Start moving forward 10cm
		comm.send("Ad")
		print "Message: Move Forward"
		time.sleep(0.42)
		comm.send("Cd")
		print "Message: Stop"

	def forward50(comm):
		# Start moving forward 50cm
		comm.send("Ad")
		print "Message: Move Forward"
		time.sleep(2.26)
		comm.send("Cd")
		print "Message: Stop"

	def backward(comm):
		# Start moving backwards
		comm.send("Bd")
		print "Message: Move Backward"
		time.sleep(0.85)
		comm.send("Cd")
		print "Message: Stop"

	def kickAttacker(comm):
		# Start kicking once very hard
		comm.send("Dd")
		print "Message: Kick hard pls"

	def kickDefender(comm):
		# Start kicking once smoothly
		comm.send("D=")
		print "Message: Kick smooth pls"

	# Execute given task
	task = int(args.task)

	if (task == 1):
		forward10(communication)
	elif (task == 2):
		forward50(communication)
	elif (task == 3):
		backward(communication)
	elif (task == 4):
		kickAttacker(communication)
	elif (task == 5):
		kickDefender(communication)
	else:
		print "Message not recognised"

	# End
	communication.close()