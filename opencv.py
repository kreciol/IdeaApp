#! /usr/bin/env python
import cv2
import numpy
import math

from time import sleep

from messages_helper import connectToMqtt
from messages_helper import isConnected
from messages_helper import moveTo
from messages_helper import lineTo
from messages_helper import clear
table_width = 512
table_height = 512
camera_width = 1280
camera_height = 720
camera_max_angle = 67.309

def toRadians(value):
	return value / 180 * math.pi


def toDeg(value):
	return value / math.pi * 180


def getCameraAngle(value):
	virtualCameraConeBottomAngle = (180.0 - camera_max_angle) / 2.0
	virtualCameraConeBottomAngleRad = toRadians(virtualCameraConeBottomAngle)
	virtualCameraConeBottomLength = 1.084
	detectedPointPositionOnVirtualCameraCone = value / camera_width * virtualCameraConeBottomLength

	topFormula = math.sin(virtualCameraConeBottomAngleRad) * detectedPointPositionOnVirtualCameraCone
	bottomFormula = math.sqrt(1 + math.pow(detectedPointPositionOnVirtualCameraCone, 2) - (2 * math.cos(virtualCameraConeBottomAngleRad) * detectedPointPositionOnVirtualCameraCone))

	return toDeg(math.asin(topFormula / bottomFormula))

def setup_camera_capture(device):
    capture = cv2.VideoCapture(device)
    if not capture.isOpened():
        sys.stderr.write("Faled to Open Capture device. Quitting.\n")
        sys.exit(1)

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
    capture.set(cv2.CAP_PROP_BRIGHTNESS, 30.0)
    capture.set(cv2.CAP_PROP_CONTRAST, 5.0)
    capture.set(cv2.CAP_PROP_SATURATION, 200.0)
    capture.set(cv2.CAP_PROP_EXPOSURE, -6.0)
    return capture

def threshold_image(channel, minimum, maximum):
    (t, tmp) = cv2.threshold(channel,maximum,0,cv2.THRESH_TOZERO_INV)
    (t, tmp2) = cv2.threshold(tmp,minimum,255,cv2.THRESH_BINARY)

    return tmp2

def dilate(frame):
    dilateElement = numpy.ones((8,8), numpy.uint8)

    return cv2.dilate(frame, dilateElement, iterations=2)

def detect(frame):
    hsv_img = cv2.cvtColor(frame[300:320, 0:camera_width], cv2.COLOR_BGR2HSV)

    # split the video frame into color channels
    h, s, v = cv2.split(hsv_img)

    # Threshold ranges of HSV components; storing the results in place
    h = threshold_image(h, 20, 160)
    h = cv2.bitwise_not(h)
    s = threshold_image(s, 100, 255)
    v = threshold_image(v, 120, 256)

    h = dilate(h)
    s = dilate(s)
    v = dilate(v)

    # Perform an AND on HSV components to identify the laser!
    laser = cv2.bitwise_and(h, v)
    laser = cv2.bitwise_and(s,laser)

    return laser

def track(frame):
    countours = cv2.findContours(frame, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(countours) > 0:
        countour = max(countours, key=cv2.contourArea)
        moments = cv2.moments(countour)
        
        if  moments["m00"] > 0:
            return int(moments["m10"] / moments["m00"])

    return -1

def drawPoint(a1, a2):
	x1 = 0.0
	y1 = 512.0
	x2 = 512.0
	y2 = 512.0

	alfa1 = 180.0 - (camera_max_angle - getCameraAngle(a1))
	alfa2 = getCameraAngle(a2)

	tan1 = math.tan(toRadians(alfa1))
	tan2 = math.tan(toRadians(alfa2))

	b1 = y1 - tan1*x1
	b2 = y2 - tan2*x2

	x = (b2 - b1) / (tan1 - tan2)
	y = tan1*x + b1

	y = 512.0 - y

	# //TODO;tutaj
	lineTo(x, y)
	
	point = int(x), int(y)
	
	# image = numpy.zeros((512, 512, 3), numpy.uint8)
	# image[:] = (255, 255, 255)
	# cv2.circle(image,  point,  2, (0, 0, 0), 2)
	# cv2.imshow('Table', image)

	return -1

def getFrame(capture):
    success, frame = capture.read()

    if not success:
        sys.stderr.write("Could not read camera frame. Quitting\n")
        sys.exit(1)
    
    return frame

def handle_quit(delay=10):
    """Quit the program if the user presses "Esc" or "q"."""
    key = cv2.waitKey(delay)
    c = chr(key & 255)
    if c in ['q', 'Q', chr(27)]:
        sys.exit(0)

print('Starting WALLe cameras')

connectToMqtt('iot.eclipse.org')

print("MQTT: Waiting for connection ", end="")
while (isConnected() == False):
    print(".", end="")
    sleep(0.1)

cv2.namedWindow('Laser1')
cv2.namedWindow('Laser2')
cv2.namedWindow('Table')

capture1 = setup_camera_capture(0)
capture2 = setup_camera_capture(1)

while True:
    frame1 = getFrame(capture1)
    frame2 = getFrame(capture2)

    laser1 = detect(frame1)
    laser2 = detect(frame2)

    cv2.imshow('Laser1', frame1)
    cv2.imshow('Laser2', frame2)

    a1 = track(laser1)
    a2 = track(laser2)

    drawPoint(a1, a2)

    handle_quit()

