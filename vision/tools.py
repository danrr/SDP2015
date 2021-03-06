# -*- coding: utf-8 -*-
import numpy as np
import cv2
import json
import socket
import os
import cPickle

PATH = os.path.dirname(os.path.realpath(__file__))
BLACK = (0, 0, 0)

# HSV Colors
WHITE_LOWER = np.array([1, 0, 100])
WHITE_HIGHER = np.array([36, 255, 255])

BLUE_LOWER = np.array([95., 50., 50.])
BLUE_HIGHER = np.array([110., 255., 255.])

RED_LOWER = np.array([0, 240, 140])
RED_HIGHER = np.array([9, 255, 255])

YELLOW_LOWER = np.array([9, 50, 50])
YELLOW_HIGHER = np.array([11, 255, 255])

PITCHES = ['Pitch_0', 'Pitch_1']


def get_zones(width, height, filename=PATH+'/calibrations/croppings.json', pitch=0):
    calibration = get_croppings(filename, pitch)
    zones_poly = [calibration[key] for key in ['Zone_0', 'Zone_1', 'Zone_2', 'Zone_3']]

    maxes = [max(zone, key=lambda x: x[0])[0] for zone in zones_poly[:3]]
    mins = [min(zone, key=lambda x: x[0])[0] for zone in zones_poly[1:]]
    mids = [(maxes[i] + mins[i]) / 2 for i in range(3)]
    mids.append(0)
    mids.append(width)
    mids.sort()
    return [(mids[i], mids[i+1], 0, height) for i in range(4)]


def get_croppings(filename=PATH+'/calibrations/croppings.json', pitch=0):
    croppings = get_json(filename)
    return croppings[PITCHES[pitch]]


def get_json(filename=PATH+'/calibrations/calibrations.json'):
    _file = open(filename, 'r')
    content = json.loads(_file.read())
    _file.close()
    return content


def get_radial_data(pitch=0, filename=PATH+'/calibrations/undistort.txt'):
    _file = open(filename, 'r')
    #pickles/serializes the data in the file undistort
    data = cPickle.load(_file)
    _file.close()
    return data[pitch]


def get_colors(pitch=0, filename=PATH+'/calibrations/calibrations_user.json'):
    """
    Get colros from the JSON calibration file.
    Json file : specific - values in this column will be overriden by exiting application
                default - values that was found to be working the best, backup
    Converts all
    """
    # If calibrations_user doesn't exist, then create calibrations_user
    # with the default values copied from calibrations.json
    if not os.path.isfile(filename):
        default_json_content = get_json(PATH+'/calibrations/calibrations.json')
        write_json(filename, default_json_content)

    json_content = get_json(filename)
    machine_name = socket.gethostname().split('.')[0]
    pitch_name = 'PITCH0' if pitch == 0 else 'PITCH1'

    #these can be overriden when exiting gui
    current = json_content['specific'][pitch_name]
    #to use default which will not be overriden
    #current = json_content['default'][pitch_name]

    #Check if any of the max values is equal to 0 if so use default values instead, this can signal an error happening
    for key in current:
        key_dict = current[key]
        if 'max' in key_dict:
            if ((key_dict['max'][0] == 0) or (key_dict['max'][1] == 0) or (key_dict['max'][2] == 0)):
                current = json_content['default'][pitch_name]
                break
        


    # convert mins and maxes into np.array
    for key in current:
        key_dict = current[key]
        if 'min' in key_dict:
            key_dict['min'] = np.array(tuple(key_dict['min']))
        if 'max' in key_dict:
            key_dict['max'] = np.array(tuple(key_dict['max']))

    return current


def save_colors(pitch, colors, filename=PATH+'/calibrations/calibrations_user.json'):

    json_content = get_json(filename)
    machine_name = socket.gethostname().split('.')[0]
    pitch_name = 'PITCH0' if pitch == 0 else 'PITCH1'

    # convert np.arrays into lists
    for key in colors:
        key_dict = colors[key]
        if 'min' in key_dict:
            key_dict['min'] = list(key_dict['min'])
        if 'max' in key_dict:
            key_dict['max'] = list(key_dict['max'])
    #store values in 'specific' comlumn
    json_content['specific'][pitch_name].update(colors)
    #backup default should be used only for one time calibration and the values duplicated to specific
    #json_content['default'][pitch_name].update(colors)

    write_json(filename, json_content)


def save_croppings(pitch, data, filename=PATH+'/calibrations/croppings.json'):
    """
    Open the current croppings file and only change the croppings
    for the relevant pitch.
    """
    croppings = get_json(filename)
    croppings[PITCHES[pitch]] = data
    write_json(filename, croppings)


def write_json(filename=PATH+'/calibrations/calibrations.json', data={}):
    _file = open(filename, 'w')
    _file.write(json.dumps(data))
    _file.close()

#Returns area enclosed in the polygon by the red dots, rest of the image will be zeroes
def mask_pitch(frame, points):
    """
    Hue : the color type (such as red, blue, or yellow).
        Ranges from 0 to 360° in most applications. (each value corresponds to one color : 0 is red, 45 is a shade of orange and 55 is a shade of yellow).
    Saturation:the intensity of the color.
        Ranges from 0 to 100% (0 means no color, that is a shade of grey between black and white; 100 means intense color).
        Also sometimes called the "purity" by analogy to the colorimetric quantities excitation purity.
    Brightness (or Value) : the brightness of the color.
        Ranges from 0 to 100% (0 is always black; depending on the saturation, 100 may be white or a more or less saturated color).
    """
    mask = frame.copy()
    #create a numpy array
    points = np.array(points, np.int32)
    #draws filled convex polygon
    cv2.fillConvexPoly(mask, points, BLACK)
    #converts into HSV
    hsv_mask = cv2.cvtColor(mask, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_mask, (0, 0, 0), (0, 0, 0))
    #ands frame with itself wherever the mask isn't 0
    m = cv2.bitwise_and(frame, frame, mask=mask)
    return cv2.bitwise_and(frame, frame, mask=mask)


def find_extremes(coords):
    #Returns cooridantes to specifiy a square enclosing the pitch based on croppings
    left = min(coords, key=lambda x: x[0])[0]
    right = max(coords, key=lambda x: x[0])[0]
    top = min(coords, key=lambda x: x[1])[1]
    bottom = max(coords, key=lambda x: x[1])[1]
    return (left, right, top, bottom)

#returns coordinates of the the image to make it square
#Usage : frame is a mask, polygon enclosed by the red dots,
#this functions encloses it in a minimum square
def find_crop_coordinates(frame, keypoints=None, width=520, height=285):
    """
    Get crop coordinated for the actual image based on masked version.

    Params:
        [int] width     fixed width of the image to crop to
        [int[ height    fixed height of the image to crop to

    Returns:
        A 4-tuple with crop values
    """
    frame_height, frame_width, channels = frame.shape
    if frame_width < width or frame_height < height:
        print 'get_crop_coordinates:', 'Requested size of the frame is smaller than the original frame'
        return frame

    if not keypoints:
        # Smoothen and apply white mask
        mask = mask_field(normalize(frame))

        # Get FAST detection of features
        fast = cv2.FastFeatureDetector()

        # get keypoints - list of Keypoints with x/y coordinates
        kp = fast.detect(mask, None)

        x_min = min(kp, key=lambda x: x.pt[0]).pt[0]
        y_min = min(kp, key=lambda x: x.pt[1]).pt[1]
        x_max = max(kp, key=lambda x: x.pt[0]).pt[0]
        y_max = max(kp, key=lambda x: x.pt[1]).pt[1]

    else:
        x_min = min(keypoints, key=lambda x: x[0])[0]
        y_min = min(keypoints, key=lambda x: x[1])[1]
        x_max = max(keypoints, key=lambda x: x[0])[0]
        y_max = max(keypoints, key=lambda x: x[1])[1]

    x_delta = x_max - x_min
    y_delta = y_max - y_min

    # x_remaining = max(0, (width - x_delta) / 2)
    # y_remaining = max(0, (height - y_delta) / 2)

    return (
        x_min, x_max,
        y_min, y_max)


def crop(frame, size=None):
    """
    Crop a frame given the size.
    If size not provided, attempt to extract the field and crop.

    Params:
        [OpenCV Frame] frame    frame to crop
        [(x1,x2,y1,y2)] size
    """
    # if not size or not len(size):
    #     x_min, x_max, y_min, y_max = get_crop_coordinates(frame)
    # else:
    # print 'SIZE:', size
    x_min, x_max, y_min, y_max = size
    return frame[y_min:y_max, x_min:x_max]
