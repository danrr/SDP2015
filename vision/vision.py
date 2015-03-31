import cv2
import tools
from tracker import BallTracker, RobotTracker
from multiprocessing import Process, Queue
from colors import BGR_COMMON
from collections import namedtuple
import numpy as np
from findHSV import CalibrationGUI
from collections import deque
from Polygon.cPolygon import Polygon
from random import randint

TEAM_COLORS = set(['yellow', 'blue'])
SIDES = ['left', 'right']
PITCHES = [0, 1]

PROCESSING_DEBUG = False

Center = namedtuple('Center', 'x y')


class Vision:
    """
    Locate objects on the pitch.
    """

    def __init__(self, pitch, color, our_side, frame_shape, frame_center, calibration):
        """
        Initialize the vision system.

        Params:
            [int] pitch         pitch number (0 or 1)
            [string] color      color of our robot
            [string] our_side   our side
        """
        self.pitch = pitch
        self.color = color
        self.our_side = our_side
        self.frame_center = frame_center

        height, width, channels = frame_shape

        # Find the zone division
        self.zones = zones = self._get_zones(width, height)

        opponent_color = self._get_opponent_color(color)

        if our_side == 'left':
            self.us = [
                RobotTracker(
                    color=color, crop=zones[0], offset=zones[0][0], pitch=pitch,
                    name='Our Defender', calibration=calibration),  # defender
                RobotTracker(
                    color=color, crop=zones[2], offset=zones[2][0], pitch=pitch,
                    name='Our Attacker', calibration=calibration)  # attacker
            ]

            self.opponents = [
                RobotTracker(
                    color=opponent_color, crop=zones[3], offset=zones[3][0], pitch=pitch,
                    name='Their Defender', calibration=calibration),
                RobotTracker(
                    color=opponent_color, crop=zones[1], offset=zones[1][0], pitch=pitch,
                    name='Their Attacker', calibration=calibration)

            ]
        else:
            self.us = [
                RobotTracker(
                    color=color, crop=zones[3], offset=zones[3][0], pitch=pitch,
                    name='Our Defender', calibration=calibration),
                RobotTracker(
                    color=color, crop=zones[1], offset=zones[1][0], pitch=pitch,
                    name='Our Attacker', calibration=calibration)
            ]

            self.opponents = [
                RobotTracker(
                    color=opponent_color, crop=zones[0], offset=zones[0][0], pitch=pitch,
                    name='Their Defender', calibration=calibration),  # defender
                RobotTracker(
                    color=opponent_color, crop=zones[2], offset=zones[2][0], pitch=pitch,
                    name='Their Attacker', calibration=calibration)
            ]

        # Set up trackers
        self.ball_tracker = BallTracker(
            (0, width, 0, height), 0, pitch, calibration)

    def _get_zones(self, width, height):
        return [(val[0], val[1], 0, height) for val in tools.get_zones(width, height, pitch=self.pitch)]

    @staticmethod
    def _get_opponent_color(our_color):
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
        coefficient = robot_height / plane_height

        x = point[0]
        y = point[1]

        dist_x = float(x - self.frame_center[0])
        dist_y = float(y - self.frame_center[1])

        delta_x = dist_x * coefficient
        delta_y = dist_y * coefficient

        return int(x - delta_x), int(y - delta_y)

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
            Process(target=obj.find, args=(frame, queues[i])) for (i, obj) in enumerate(objects)]

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

    @staticmethod
    def to_info(args, height):
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


class Camera(object):
    """
    Camera access wrapper.
    """

    def __init__(self, port=0, pitch=0):
        self.capture = cv2.VideoCapture(port)
        calibration = tools.get_croppings(pitch=pitch)
        self.crop_values = tools.find_extremes(calibration['outline'])
        # Parameters used to fix radial distortion
        radial_data = tools.get_radial_data()
        self.nc_matrix = radial_data['new_camera_matrix']
        self.c_matrix = radial_data['camera_matrix']
        self.dist = radial_data['dist']
        # used for calibration loop
        self.frameNo = 0

    def get_frame(self, calibration_loop=False):
        """
        Retrieve a frame from the camera.

        Returns the frame if available, otherwise returns None.
        """
        # counts the frame no's for video loop
        if calibration_loop:
            status, frame = True, cv2.imread('img/test' + str(self.frameNo) + '.jpg')
            if self.frameNo == 19:
                self.frameNo = 0
            else:
                self.frameNo += 1
        else:
            status, frame = self.capture.read()
        frame = self.fix_radial_distortion(frame)
        if status:
            return frame[
                self.crop_values[2]:self.crop_values[3],
                self.crop_values[0]:self.crop_values[1]
            ]

    def fix_radial_distortion(self, frame):
        return cv2.undistort(
            frame, self.c_matrix, self.dist, None, self.nc_matrix)

    def get_adjusted_center(self):
        return 320 - self.crop_values[0], 240 - self.crop_values[2]

class Calibrate(object):
    def __init__(self, calibration_gui, jitterQueue):
        self.calibration_gui = calibration_gui
        self.values = [(self.calibration_gui.get_trackbar_positions(), sum(jitterQueue))]
        self.colour = self.calibration_gui.color
        self.descentPoint = 0
        self.setting = "LH"
        self.direction = -1
        self.alpha = 1

    def __del__(self):
        self.values = None

    def update(self, jitterQueue):
        jitterValue = sum(jitterQueue)
        if self.colour != self.calibration_gui.color:
            self.values = [(self.calibration_gui.get_trackbar_positions(), sum(jitterQueue))]
            self.colour = self.calibration_gui.color
            self.descentPoint = 0
            self.setting = "LH"
            self.direction = -1
            self.alpha = 1
            return

        values = self.calibration_gui.get_trackbar_positions()
        if self.values[self.descentPoint][1] > jitterValue:
            values = self.values[len(self.values)-1][0]
            self.descentPoint = len(self.values)
            values[self.setting] += self.direction*self.alpha
        elif self.values[self.descentPoint][1] < jitterValue:
            values = self.values[self.descentPoint][0]
            self.descentPoint = len(self.values)
            if self.direction > 0:
                self.setting = self.get_next_setting(self.setting)
                self.direction = -1
            else:
                self.direction = 1
        print values
        self.values.append((values, jitterValue))
        self.calibration_gui.set_trackbar_positions(values)

    def get_next_setting(self, currentSetting):
        CONTROLS = ["LH", "UH", "LS", "US", "LV", "UV", "CT", "BL"]

        index = CONTROLS.index(currentSetting)
        return CONTROLS[(index+1) % 8]


class GUI(object):
    VISION = 'SUCH VISION'
    BG_SUB = 'BG Subtract'
    NORMALIZE = 'Normalize  '
    COMMS = 'Communications on/off '
    CALIB_LOOP = 'Calibration Loop'
    CALIBRATE = 'Calibrate'

    def nothing(self, x):
        pass

    def __init__(self, calibration, arduino, pitch, capture):
        self.zones = None
        self.calibration_gui = CalibrationGUI(calibration)
        self.arduino = arduino
        self.pitch = pitch
        self.capture = capture
        # identifies whteher we are using loop or real video feed
        self.calibration_loop = False
        self.calibrator = None
        self.can_see_all = False

        cv2.namedWindow(self.VISION)

        cv2.createTrackbar(self.BG_SUB, self.VISION, 0, 1, self.nothing)
        cv2.createTrackbar(self.NORMALIZE, self.VISION, 0, 1, self.nothing)
        # If no connection with arduino can be estabilished and slider is moved to 1
        # it will incorrectly reflect the communications state
        cv2.createTrackbar(
            self.COMMS, self.VISION, self.arduino.comms, 1, lambda x: self.arduino.setComms(x))
        cv2.createTrackbar(self.CALIB_LOOP, self.VISION, 0, 1, lambda x: self.set_calibration_loop(x))
        cv2.createTrackbar(self.CALIBRATE, self.VISION, 0, 1, lambda x: self.calibrate(x))
        # 20 zeroes, arbitrary number
        self.jitterQueue = deque([0, 0, 0, 0, 0])

    @staticmethod
    def to_info(args):
        """
        Convert a tuple into a vector

        Return a Vector
        """
        x, y, angle, velocity = None, None, None, None
        if args is not None:
            if 'location' in args:
                x = args['location'][0] if args['location'] is not None else None
                y = args['location'][1] if args['location'] is not None else None

            elif 'x' in args and 'y' in args:
                x = args['x']
                y = args['y']

            if 'angle' in args:
                angle = args['angle']

            if 'velocity' in args:
                velocity = args['velocity']

        return {'x': x, 'y': y, 'angle': angle, 'velocity': velocity}

    @staticmethod
    def cast_binary(x):
        return x == 1

    # For all robots/plates on the field takes how much they have moved since the last frame
    # Computes the jitter factor
    def calcJitterFactor(self, model_positions):
        for name, info in model_positions.iteritems():
            if name != 'ball':
                if not (info.x is None) and not (info.y is None):
                    self.jitterQueue.append(abs(info.velocity))
                    self.jitterQueue.popleft()


    def draw(self, frame, model_positions, regular_positions, fps,
             dState, grabbers, frameNo, our_color, our_side,
             key=None, preprocess=None):
        """
        Draw information onto the GUI given positions from the vision and post processing.

        NOTE: model_positions contains coordinates with y coordinate reversed!
        """
        # Get general information about the frame
        frame_height, frame_width, channels = frame.shape

        # Draw the calibration gui
        self.calibration_gui.show(frame, key)
        # Draw dividors for the zones
        self.draw_zones(frame, frame_width, frame_height)

        their_color = list(TEAM_COLORS - set([our_color]))[0]

        key_color_pairs = zip(
            ['our_defender', 'their_defender', 'our_attacker', 'their_attacker'],
            [our_color, their_color] * 2)

        self.draw_ball(frame, regular_positions['ball'])

        for key, color in key_color_pairs:
            self.draw_robot(frame, regular_positions[key], color)

        # Draw fps on the canvas
        if fps is not None:
            self.draw_text(frame, 'FPS: %.1f' % fps, 0, 10, BGR_COMMON['green'], 1)

        # calculates the jitter factor per 20 frames, 19 is just an arbitrary number
        self.calcJitterFactor(model_positions)
        jitterFactor = 0
        for jitterFrame in self.jitterQueue:
            jitterFactor += jitterFrame
        # indicates whether we can see all robots
        self.can_see_all = True
        for key in ['our_defender', 'our_attacker', 'their_defender', 'their_attacker']:
            for label in regular_positions[key]:
                # if some of box direction angle is none we cant see the plate
                if regular_positions[key][label] is None:
                    self.can_see_all = False
                # check if the box is of a reasonable size
                elif label == 'box':
                    area = Polygon(regular_positions[key][label]).area()
                    # guessed reasonable number
                    if area < 60:
                        self.can_see_all = False

        if self.can_see_all:
            self.draw_text(frame, '%.1f JITTERS' % jitterFactor, 415, 15, BGR_COMMON['green'], thickness=1.5, size=0.4)
        else:
            self.draw_text(frame, 'CANT SEE', 415, 15, BGR_COMMON['red'], thickness=1.5, size=0.4)

        if preprocess is not None:
            preprocess['normalize'] = self.cast_binary(
                cv2.getTrackbarPos(self.NORMALIZE, self.VISION))
            preprocess['background_sub'] = self.cast_binary(
                cv2.getTrackbarPos(self.BG_SUB, self.VISION))

        if grabbers:
            self.draw_grabbers(frame, grabbers, frame_height)

        # Extend image downwards and draw states.
        blank = np.zeros_like(frame)[:200, :, :]
        frame_with_blank = np.vstack((frame, blank))
        self.draw_states(frame_with_blank, dState, (frame_width, frame_height))

        if model_positions and regular_positions:
            for key in ['ball', 'our_defender', 'our_attacker', 'their_defender', 'their_attacker']:
                if model_positions[key] and regular_positions[key]:
                    self.data_text(
                        frame_with_blank, (frame_width, frame_height), our_side, key,
                        model_positions[key].x, model_positions[key].y,
                        model_positions[key].angle, model_positions[key].velocity)
                    self.draw_velocity(
                        frame_with_blank, (frame_width, frame_height),
                        model_positions[key].x, model_positions[key].y,
                        model_positions[key].angle, model_positions[key].velocity)

        cv2.imshow(self.VISION, frame_with_blank)

        if frameNo % 5 == 0 and self.calibrator:
            self.calibrator.update(self.jitterQueue)


    def draw_zones(self, frame, width, height):
        # Re-initialize zones in case they have not been initalized
        if self.zones is None:
            self.zones = tools.get_zones(width, height, pitch=self.pitch)

        for zone in self.zones:
            cv2.line(frame, (zone[1], 0), (zone[1], height), BGR_COMMON['orange'], 1)

    def draw_ball(self, frame, position_dict):
        if position_dict and position_dict['x'] and position_dict['y']:
            frame_height, frame_width, _ = frame.shape
            self.draw_line(
                frame, ((int(position_dict['x']), 0), (int(position_dict['x']), frame_height)), 1)
            self.draw_line(
                frame, ((0, int(position_dict['y'])), (frame_width, int(position_dict['y']))), 1)

    @staticmethod
    def draw_dot(frame, location):
        if location is not None:
            cv2.circle(frame, location, 2, BGR_COMMON['white'], 1)

    @staticmethod
    def draw_robot(frame, position_dict, color):
        if position_dict['box']:
            cv2.polylines(frame, [np.array(position_dict['box'])], True, BGR_COMMON[color], 2)
        if position_dict['front']:
            p1 = (position_dict['front'][0][0], position_dict['front'][0][1])
            p2 = (position_dict['front'][1][0], position_dict['front'][1][1])
            cv2.circle(frame, p1, 3, BGR_COMMON['white'], -1)
            cv2.circle(frame, p2, 3, BGR_COMMON['white'], -1)
            cv2.line(frame, p1, p2, BGR_COMMON['red'], 2)

        if position_dict['dot']:
            cv2.circle(
                frame, (int(position_dict['dot'][0]), int(position_dict['dot'][1])),
                4, BGR_COMMON['black'], -1)

        if position_dict['direction']:
            cv2.line(
                frame, position_dict['direction'][0], position_dict['direction'][1],
                BGR_COMMON['orange'], 2)

    @staticmethod
    def draw_line(frame, points, thickness=2):
        if points is not None:
            cv2.line(frame, points[0], points[1], BGR_COMMON['red'], thickness)

    def data_text(self, frame, frame_offset, our_side, text, x, y, angle, velocity):
        if x is not None and y is not None:
            frame_width, frame_height = frame_offset
            if text == "ball":
                y_offset = frame_height + 130
                draw_x = 30
            else:
                x_main = lambda zz: (frame_width / 4) * zz
                x_offset = 30
                y_offset = frame_height + 20

                if text == "our_defender":
                    draw_x = x_main(0) + x_offset
                elif text == "our_attacker":
                    draw_x = x_main(2) + x_offset
                elif text == "their_defender":
                    draw_x = x_main(3) + x_offset
                else:
                    draw_x = x_main(1) + x_offset

                if our_side == "right":
                    draw_x = frame_width - draw_x - 80

            self.draw_text(frame, text, draw_x, y_offset)
            self.draw_text(frame, 'x: %.2f' % x, draw_x, y_offset + 10)
            self.draw_text(frame, 'y: %.2f' % y, draw_x, y_offset + 20)

            if angle is not None:
                self.draw_text(frame, 'angle: %.2f' % angle, draw_x, y_offset + 30)

            if velocity is not None:
                self.draw_text(frame, 'velocity: %.2f' % velocity, draw_x, y_offset + 40)

    @staticmethod
    def draw_text(frame, text, x, y, color=BGR_COMMON['green'], thickness=1.3, size=0.3, ):
        if x is not None and y is not None:
            cv2.putText(
                frame, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness)

    @staticmethod
    def draw_grabbers(frame, grabbers, height):
        for colour, [area] in grabbers.items():
            area = [(x, height - y) for x, y in area]
            area = [(int(x) if x > -1 else 0, int(y) if y > -1 else 0) for x, y in area]
            cv2.polylines(frame, [np.array(area)], True, BGR_COMMON[colour], 1)

    def draw_velocity(self, frame, frame_offset, x, y, angle, vel, scale=10):
        if not (None in [frame, x, y, angle, vel]) and vel is not 0:
            frame_width, frame_height = frame_offset
            r = vel * scale
            y = frame_height - y
            start_point = (x, y)
            end_point = (x + r * np.cos(angle), y - r * np.sin(angle))
            self.draw_line(frame, (start_point, end_point))

    def draw_states(self, frame, dState, frame_offset):
        frame_width, frame_height = frame_offset
        x_main = lambda zz: (frame_width / 4) * zz
        x_offset = 20
        y_offset = frame_height + 140

        self.draw_text(frame, "Defender State:", x_main(2) + x_offset, y_offset, size=0.6)
        self.draw_text(frame, dState[0], x_main(2) + x_offset, y_offset + 15, size=0.6)
        self.draw_text(frame, dState[1], x_main(2) + x_offset, y_offset + 30, size=0.6)

    # sets whether we are using calibration loop or real video feed
    def set_calibration_loop(self, value):
        self.calibration_loop = value
        # capture video
        # first frame is usually distorted, want to ditch it
        ret, frame = self.capture.read()
        # 20 is number of images we have captured
        if value:
            i = 0
            while i != 20:
                ret, frame = self.capture.read()
                cv2.imwrite('img/test' + str(i) + '.jpg', frame)

    def calibrate(self, value):
        if value:
            self.calibrator = Calibrate(self.calibration_gui, self.jitterQueue)
        elif self.calibrator:
            self.calibrator = None

if __name__ == '__main__':
    tableNumber = 0
    video_port = 0
    name = "Team 14"  # Name of the main GUI frame
    color = "blue"
    our_side = "left"
    calibration = tools.get_colors(tableNumber)

    # Startup vision class (responsible for all vision-related code AND draws GUI for feedback)
    vision = Vision(tableNumber, name, color, our_side, calibration)
    vision.loop()