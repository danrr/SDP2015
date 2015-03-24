import atexit 
import curses
import argparse
import serial
import time

def cleanup(): 
    curses.nocbreak() 
    stdscr.keypad(0) 
    curses.echo() 
    curses.endwin() 

def draw_data(screen, data):
    screen.clear()
    size = screen.getmaxyx()

    # if the list is too big to fit on screen, only display the last items...

    if (len(data) > size[0]):
        data = data[-size[0]:]

    for row in range(0, len(data)):
        screen.addstr(row, 1, data[row])
    screen.refresh()

def draw_status(screen, message):
    screen.clear()
    screen.box() 
    screen.addstr(1, 2, message)
    screen.refresh()

def init_curses():
    stdscr = curses.initscr() 
    curses.noecho() 
    curses.cbreak() 
    stdscr.nodelay(1)
    stdscr.keypad(1)
    curses.start_color() 
    curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLACK) 
    curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    stdscr.bkgd(curses.color_pair(1)) 
    stdscr.refresh() 
    return stdscr 

def init_conn_window(size):
    conn_window = curses.newwin(3, size[1], 0, 0) 
    conn_window.bkgd(curses.color_pair(2)) 
    return conn_window

def init_cmd_window(size):
    cmd_window = curses.newwin(3, size[1], size[0] - 3, 0) 
    cmd_window.bkgd(curses.color_pair(1)) 
    return cmd_window

def init_data_window(size):
    data_window = curses.newwin(size[0] - 6, size[1], 3, 0) 
    data_window.bkgd(curses.color_pair(1))
    return data_window

def wait_and_exit():
    stdscr.nodelay(0)
    stdscr.getch()
    exit()

def parse_command(command):
    cmd = 0
    value = 0
    description = ""
    try:
        if (command.startswith("forward")):
            cmd = ord('W')
            value = int(command.replace("forward", ""))
            description = "Forward " + str(value)
        elif (command.startswith("backward")):
            cmd = ord("S")
            value = int(command.replace("backward", ""))
            description = "Backward " + str(value)
        elif (command.startswith("stop")):
            cmd = ord(" ")
            description = "Stop"
        elif (command.startswith("heartbeat")):
            cmd = ord("L")
            value = int(command.replace("heartbeat", ""))
            description = "Heartbeat " + str(value)
        elif (command.startswith("kick")):
            cmd = ord("Q")
            value = int(command.replace("kick", ""))
            description = "Kick " + str(value)
        elif (command.startswith("grabber open")):
            cmd = ord("Z")
            description = "Grabber Open"
            value = int(command.replace("grabber open", ""))
        elif (command.startswith("grabber close")):
            cmd = ord("X")
            description = "Grabber Close"  
            value = int(command.replace("grabber close", ""))
        elif (command.startswith("turn left")):
            cmd = ord("A")
            value = int(command.replace("turn left", ""))
            description = "Turn Left"
        elif (command.startswith("turn right")):
            cmd = ord("D")
            value = int(command.replace("turn right", ""))
            description = "Turn Right"              
        elif (command.startswith("get heading")):
            cmd = ord("U")
            description = "Get Heading"
        elif (command.startswith("get timing")):
            cmd = ord("T")
            description = "Get Timing" 
            value = int(command.replace("get timing", ""))      
        elif (command.startswith("strafe left")):
            cmd = ord("C")
            value = int(command.replace("strafe left", ""))
            description = "Strafe Left " + str(value)  
        elif (command.startswith("strafe right")):
            cmd = ord("V")
            value = int(command.replace("strafe right", ""))
            description = "Strafe Right " + str(value)
        elif (command.startswith("get rotary")):
            cmd = ord("R")
            value = int(command.replace("get rotary", ""))
            description = "Get Rotary " + str(value)                  
    except:
        cmd == 0

    if (cmd == 0):
        return [0, 0, "Could not parse the command \"" + command + "\"."]
    else:
        return [cmd, value, description]

atexit.register(cleanup) 
stdscr = init_curses()

size = stdscr.getmaxyx()
conn_window = init_conn_window(size)
cmd_window = init_cmd_window(size)
data_window = init_data_window(size)

parser = argparse.ArgumentParser()
parser.add_argument("--port", help="Serial port location", default="")
args = parser.parse_args()

conn_success = False

f = open("log.txt", "w+")

draw_status(cmd_window, "")

ser = serial.Serial()

if (args.port == ""):
    draw_status(conn_window, "No port specified. Press any key to exit.")
    wait_and_exit()
else:
    draw_status(conn_window, "Opening \"" + args.port + "\"...")
    try:
        ser = serial.Serial(args.port, 115200, timeout=1)
        draw_status(conn_window, "Connected to \"" + args.port + "\".")
        conn_success = True
    except:
        draw_status(conn_window, "Could not open the serial port specified. Press any key to exit.")

if (not conn_success):
    wait_and_exit()

 
data = [""]
draw_data(data_window, data)

current_command = ""
while True:

    # if more characters are available, then get the next one

    c = stdscr.getch() 
    if (c != curses.ERR):
        if (chr(c) == '\n'):
            if (current_command != ""):
                command = parse_command(current_command)
                data.append(hex(command[0]) + " " + hex(command[1]) + " (" + command[2] + ")")

                try:
                    ser.write(chr(command[0]))
                    ser.write(chr(command[1]))
                    f.write(hex(command[0]) + ' ' + hex(command[1]) + '\n')
                except Exception as e:
                    draw_status(conn_window, e.message)
                draw_data(data_window, data)
            current_command = ""
        elif (c == 127 or c == 8):
            if (current_command > 0):
                current_command = current_command[:-1]
        else:
            current_command += chr(c)
        draw_status(cmd_window, current_command)
    
    # also check if input is available from the serial connection
    while (ser.inWaiting()):
        response = ord(ser.read())
        data.append("-> " + str(response))
        f.write("-> " + str(response) + '\n')
    draw_data(data_window, data)

