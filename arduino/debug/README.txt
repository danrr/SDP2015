Debug.py and Flood.py are two debugging programs for the robot.

Debug simplifies the process of sending commands to the robot, and uses a more natural way of representing both commands and data. Flood.py will take the first command, replicate it infinitely, and flood the serial comms with it, whilst displaying any received messages on the terminal.

To run the program, use

	python debug.py --port [path/to/device]
or
	python flood.py —-port [path/to/device]

where [path/to/device] is the serial port used by the Arduino, eg. /dev/tty.usbmodem000001.

Sending commands is done by typing the command in question, eg. "forward" followed by a space and a value if needed. For example,

	forward 100

will send the values 0x57 0x64 down the serial link. The debugger will output the decimal value of any data received from the Arduino, as well as the hex representation of the data sent to the Arduino. It will also save a copy of the communications in a log file - log.txt for further analysis.


The commands currently available are as follows:

forward [value (as percentage)]
backward [value (as percentage)]
stop
kick [value (as percentage)]
heartbeat [message]
grabber [open | close] [optional value]
strafe [left | right] [value]
turn [left | right] [value (in degrees)]

