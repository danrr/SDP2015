# This file has been rewritten entirely, using the existing code.

import cv2
import tools
import numpy as np
from threading import Thread

# NOTE: this file uses files the following:
# callibrations (folder)
# tools.py

# TODO: recognise objects on the table and provide their coordinates to the word-model
class Vision:

    def __init__(self, tableNumber, video_port, name):

        # Set up camera for capturing frames
        self.camera = Camera(tableNumber, video_port)

        # Set up GUI for displaying what's going on
        self.gui = GUI(name, self.camera)


    def loop(self):
        # TODO: exit on pressing escape
        while (cv2.waitKey(1) != 27):
            self.camera.update()
            self.gui.update()

        self.gui.end()
        self.camera.end()

# Singleton class responsible for capturing frames
class Camera:

    frame = None # Most recent frame from camera     

    def __init__(self, tableNumber, video_port):

        self.capture = cv2.VideoCapture(0) # capture-socket to receive frames from the camera

        # Parameters used to crop frame to only contain the table in view
        calibration = tools.get_croppings(pitch=tableNumber)
        self.crop_values = tools.find_extremes(calibration['outline']) # crops down to the specified table's outline

        # Parameters used to fix radial distortion
        radial_data = tools.get_radial_data()
        self.nc_matrix = radial_data['new_camera_matrix'] # new camera radial data, stored as matrix
        self.c_matrix = radial_data['camera_matrix'] # old camera radial data, stored as matrix
        self.dist = radial_data['dist'] # radial distortion

    # Updates the frame seen by the hardware-camera
    def update(self):
        (status, frame) = self.capture.read()
        frame = self.fix_radial_distortion(frame)
        if status:
            self.frame = frame[
                self.crop_values[2]:self.crop_values[3],
                self.crop_values[0]:self.crop_values[1]
            ]
        print "[Camera] seting new frame . . . "

    def end(self):
        self.capture.release()

    # Just a getter - can be refactored later
    def get_frame(self):
        return self.frame

    # Fixes fish-eye effect caused by the camera
    def fix_radial_distortion(self, frame):
        return cv2.undistort(
            frame, self.c_matrix, self.dist, None, self.nc_matrix)

# Will be useful for visualising what the robot sees and tries to act upon
class GUI:

    def __init__(self, name, camera):
        self.name   = name      # name of GUI window
        self.camera = camera    # camera from which the frames are received

    def update(self):
        print "[GUI] drawing new frame . . . "
        # Capture frame-by-frame
        frame = self.camera.get_frame()
        # Display the resulting frame
        cv2.imshow(self.name, frame)

    def stop(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':

    tableNumber = 0
    video_port = 0
    name = "Team 14" # name of the main GUI frame

    # Startup the vision. Currently it's responsible for updating the GUI (i.e showing a livestream of the table)
    vision = Vision(tableNumber, video_port, name)
    vision.loop()