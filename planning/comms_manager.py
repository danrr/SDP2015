import time


class CommunicationsManager(object):

    def __init__(self, arduino):
        self.arduino = arduino
        self.last_command = None
        self.timer = None

    def check_last_command(self, command):
        # TODO: use return thingies
        if self.timer and self.timer + 0.5 < time.clock():
            self.timer = None
            self.last_command = None
        if command == self.last_command:
            return False
        else:
            self.last_command = command
            self.timer = time.clock()
            return True

    def move_forward(self, power):
        if self.check_last_command(('W', power)):
            print "Move forward: {amount}".format(amount=power)
            self.arduino.send('W', power)

    def move_backward(self, power):
        if self.check_last_command(('S', power)):
            print "Move Backward: {amount}".format(amount=power)
            self.arduino.send('S', power)

    def strafe_right(self, power):
        if self.check_last_command('V'):
            print "Strafe right: {amount}".format(amount=power)
            self.arduino.send('V', power)

    def strafe_left(self, power):
        if self.check_last_command('C'):
            print "Strafe left: {amount}".format(amount=power)
            self.arduino.send('C', power)

    def turn_right(self, angle):
        if self.check_last_command('D'):
            print "Turn right: {angle}".format(angle=angle)
            self.arduino.send('D', angle / 2)

    def turn_left(self, angle):
        if self.check_last_command('A'):
            print "Turn left: {angle}".format(angle=angle)
            self.arduino.send('A', angle / 2)

    def stop(self):
        if self.check_last_command(' '):
            print "Sending stop"
            self.arduino.send(' ', 0)

    def close_grabber_center(self):
        print "Close both grabbers at once"
        self.arduino.send('X', 0)

    def close_grabber_right(self):
        print("Close right grabber first")
        self.arduino.send('X', 1)

    def close_grabber_left(self):
        print("Close left grabber first")
        self.arduino.send('X', 0)

    def open_grabber(self):
        print("Open grabber")
        self.arduino.send('Z', 0)

    def kick(self, power=100):
        print("Kick")
        self.arduino.send('Q', power)

    def shutdown(self):
        print("SHUT DOWN")
        self.arduino.send(" ", 0)
