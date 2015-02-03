import cv2
import tools
import numpy as np
from tracker import BallTracker, RobotTracker
from multiprocessing import Process, Queue
from collections import namedtuple
from threading import Thread


TEAM_COLORS = set(['yellow', 'blue'])
SIDES = ['left', 'right']
PITCHES = [0, 1]
Center = namedtuple('Center', 'x y')

class Vision:

    def __init__(self, tableNumber, name, color, our_side, calibration, video_port=0):

        # Set up camera for capturing frames
        self.camera = Camera(tableNumber, video_port)
        self.camera.update()
        # Set up GUI for displaying what's going on
        self.gui = GUI(name, self.camera)

        frame = self.camera.get_frame()
        frame_shape = frame.shape
        frame_center = self.camera.get_adjusted_center()

        self.tableNumber = tableNumber
        self.color = color
        self.our_side = our_side
        self.frame_center = frame_center
        height, width, channels = frame_shape
        self.zones = zones = self._get_zones(width, height)
        opponent_color = self._get_opponent_color(color)

        # Registering objects (trackers)
        if our_side == 'left':
            self.us = [
                RobotTracker(
                    color=color, crop=zones[0], offset=zones[0][0], pitch=tableNumber,
                    name='Our Defender', calibration=calibration),   # defender
                RobotTracker(
                    color=color, crop=zones[2], offset=zones[2][0], pitch=tableNumber,
                    name='Our Attacker', calibration=calibration)   # attacker
            ]

            self.opponents = [
                RobotTracker(
                    color=opponent_color, crop=zones[3], offset=zones[3][0], pitch=tableNumber,
                    name='Their Defender', calibration=calibration),
                RobotTracker(
                    color=opponent_color, crop=zones[1], offset=zones[1][0], pitch=tableNumber,
                    name='Their Attacker', calibration=calibration)
            ]
        else:
            self.us = [
                RobotTracker(
                    color=color, crop=zones[3], offset=zones[3][0], pitch=tableNumber,
                    name='Our Defender', calibration=calibration),
                RobotTracker(
                    color=color, crop=zones[1], offset=zones[1][0], pitch=tableNumber,
                    name='Our Attacker', calibration=calibration)
            ]

            self.opponents = [
                RobotTracker(
                    color=opponent_color, crop=zones[0], offset=zones[0][0], pitch=tableNumber,
                    name='Their Defender', calibration=calibration),   # defender
                RobotTracker(
                    color=opponent_color, crop=zones[2], offset=zones[2][0], pitch=tableNumber,
                    name='Their Attacker', calibration=calibration)
            ]

        self.ball_tracker = BallTracker(
            (0, width, 0, height), 0, tableNumber, calibration)

    def _get_zones(self, width, height):
        return [(val[0], val[1], 0, height) for val in tools.get_zones(width, height, pitch=self.tableNumber)]

    def _get_opponent_color(self, our_color):
        return (TEAM_COLORS - set([our_color])).pop()

    def locate(self, frame):
        """
        Find objects on the pitch using multiprocessing.

        Returns:
            [5-tuple] Location of the robots and the ball
        """
        # Run trackers as processes
        positions = self._run_trackers(frame)
        # Correct for perspective
        positions = self.get_adjusted_positions(positions)

        # Wrap list of positions into a dictionary
        keys = ['our_defender', 'our_attacker', 'their_defender', 'their_attacker', 'ball']
        regular_positions = dict()
        for i, key in enumerate(keys):
            regular_positions[key] = positions[i]

        # Error check we got a frame
        height, width, channels = frame.shape if frame is not None else (None, None, None)

        model_positions = {
            'our_attacker': self.to_info(positions[1], height),
            'their_attacker': self.to_info(positions[3], height),
            'our_defender': self.to_info(positions[0], height),
            'their_defender': self.to_info(positions[2], height),
            'ball': self.to_info(positions[4], height)
        }

        return model_positions, regular_positions

    def get_adjusted_point(self, point):
        """
        Given a point on the plane, calculate the adjusted point, by taking into account
        the height of the robot, the height of the camera and the distance of the point
        from the center of the lens.
        """
        plane_height = 250.0
        robot_height = 20.0
        coefficient = robot_height/plane_height

        x = point[0]
        y = point[1]

        dist_x = float(x - self.frame_center[0])
        dist_y = float(y - self.frame_center[1])

        delta_x = dist_x * coefficient
        delta_y = dist_y * coefficient

        return (int(x-delta_x), int(y-delta_y))

    def get_adjusted_positions(self, positions):
        try:
            for robot in range(4):
                # Adjust each corner of the plate
                for i in range(4):
                    x = positions[robot]['box'][i][0]
                    y = positions[robot]['box'][i][1]
                    positions[robot]['box'][i] = self.get_adjusted_point((x, y))

                new_direction = []
                for i in range(2):
                    # Adjust front line
                    x = positions[robot]['front'][i][0]
                    y = positions[robot]['front'][i][1]
                    positions[robot]['front'][i] = self.get_adjusted_point((x, y))

                    # Adjust direction line
                    x = positions[robot]['direction'][i][0]
                    y = positions[robot]['direction'][i][1]
                    adj_point = self.get_adjusted_point((x, y))
                    new_direction.append(adj_point)

                # Change the namedtuples used for storing direction points
                positions[robot]['direction'] = (
                    Center(new_direction[0][0], new_direction[0][1]),
                    Center(new_direction[1][0], new_direction[1][1]))

                # Adjust the center point of the plate
                x = positions[robot]['x']
                y = positions[robot]['y']
                new_point = self.get_adjusted_point((x, y))
                positions[robot]['x'] = new_point[0]
                positions[robot]['y'] = new_point[1]
        except:
            # At least one robot has not been found
            pass

        return positions

    def _run_trackers(self, frame):
        """
        Run trackers as separate processes

        Params:
            [np.frame] frame        - frame to run trackers on

        Returns:
            [5-tuple] positions     - locations of the robots and the ball
        """
        queues = [Queue() for i in range(5)]
        objects = [self.us[0], self.us[1], self.opponents[0], self.opponents[1], self.ball_tracker]

        # Define processes
        processes = [
            Process(target=obj.find, args=((frame, queues[i]))) for (i, obj) in enumerate(objects)]

        # Start processes
        for process in processes:
            process.start()

        # Find robots and ball, use queue to
        # avoid deadlock and share resources
        positions = [q.get() for q in queues]

        # terminate processes
        for process in processes:
            process.join()

        return positions

    def to_info(self, args, height):
        """
        Returns a dictionary with object position information
        """
        x, y, angle, velocity = None, None, None, None
        if args is not None:
            if 'x' in args and 'y' in args:
                x = args['x']
                y = args['y']
                if y is not None:
                    y = height - y

            if 'angle' in args:
                angle = args['angle']

            if 'velocity' in args:
                velocity = args['velocity']

        return {'x': x, 'y': y, 'angle': angle, 'velocity': velocity}

    def loop(self):

        while ((cv2.waitKey(1) & 0xFF) != 27):
            # update frame and refresh gui window
            self.camera.update()
            self.gui.update()

            # Print position and data of the ball
            print self.locate(self.camera.get_frame())[0]["ball"]


        self.gui.end()
        self.camera.end()

    def locate(self, frame):
        # Run trackers as processes
        positions = self._run_trackers(frame)
        # Correct for perspective
        positions = self.get_adjusted_positions(positions)

        # Wrap list of positions into a dictionary
        keys = ['our_defender', 'our_attacker', 'their_defender', 'their_attacker', 'ball']
        regular_positions = dict()
        for i, key in enumerate(keys):
            regular_positions[key] = positions[i]

        # Error check we got a frame
        height, width, channels = frame.shape if frame is not None else (None, None, None)

        model_positions = {
            'our_attacker': self.to_info(positions[1], height),
            'their_attacker': self.to_info(positions[3], height),
            'our_defender': self.to_info(positions[0], height),
            'their_defender': self.to_info(positions[2], height),
            'ball': self.to_info(positions[4], height)
        }

        return model_positions, regular_positions

# Singleton class responsible for capturing frames
class Camera:

    frame = None # Most recent frame from camera

    def __init__(self, tableNumber, video_port):

        self.capture = cv2.VideoCapture(video_port) # capture-socket to receive frames from the camera

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

    def end(self):
        self.capture.release()

    # Just a getter - can be refactored later
    def get_frame(self):
        return self.frame

    # Fixes fish-eye effect caused by the camera
    def fix_radial_distortion(self, frame):
        return cv2.undistort(
            frame, self.c_matrix, self.dist, None, self.nc_matrix)

    def get_adjusted_center(self):
        return (320-self.crop_values[0], 240-self.crop_values[2])

# Will be useful for visualising what the robot sees and tries to act upon
# TODO: Need to create a GUI better than previous year's team's
class GUI:

    def __init__(self, name, camera):
        self.name   = name      # name of GUI window
        self.camera = camera    # camera from which the frames are received

    def update(self):
        # Capture frame-by-frame
        frame = self.camera.get_frame()
        # Display the resulting frame
        cv2.imshow(self.name, frame)

    def stop(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':

    tableNumber = 0
    video_port = 0
    name = "Team 14" # Name of the main GUI frame
    color = "blue"
    our_side = "left"
    calibration = tools.get_colors(tableNumber)

    # Startup vision class (responsible for all vision-related code AND draws GUI for feedback)
    vision = Vision(tableNumber, name, color, our_side, calibration)
    vision.loop()