#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
import sys
import rospy
import cv2
import numpy as np
import math
import os
import sys
#import pandas as pd
from std_msgs.msg import String
from sensor_msgs.msg import Image
from swiftpro.msg import *
from cv_bridge import CvBridge, CvBridgeError
from sklearn.linear_model import LinearRegression
import threading
import time
import hsv_detection

xc = 0
yc = 0

count = 20

index = 0

xarray = np.zeros(count)
yarray = np.zeros(count)
xc_array = np.zeros(count)
yc_array = np.zeros(count)

def hsv_value():
	try:
		filename = os.environ['HOME'] + "/calibration_HSV.txt"
		with open(filename, "r") as f:
			for line in f:
				split = line.split(':')[1].split(' ')
				lower_red = split[0].split(',')
				upper_red = split[1].split(',')
				for i in range(3):
					lower_red[i] = int(lower_red[i])
					upper_red[i] = int(upper_red[i])

		lower_red = np.array(lower_red)
		upper_red = np.array(upper_red)
	except:
		raise IOError('could not find hsv_value file : {},please execute #13 command automatically '
					  'generate this file'.format(filename))

	return lower_red,upper_red

def image_callback():
	global xc, yc

	lower_red, upper_red = hsv_value()
	capture = cv2.VideoCapture(0)

	while True:
		ret,frame = capture.read()
		cv2.imshow("win1", frame)
		cv2.waitKey(1)
		cv_image2 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(cv_image2, lower_red, upper_red)
		# smooth and clean noise
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
		mask = cv2.GaussianBlur(mask, (5,5), 0)
		# detect contour
		cv2.imshow("win2", mask)
		cv2.waitKey(1)
		_, contours, hier = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		size = []
		size_max = 0
		for i, c in enumerate(contours):
			rect = cv2.minAreaRect(c)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
			y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
			w = math.sqrt((box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2)
			h = math.sqrt((box[0][0] - box[3][0])**2 + (box[0][1] - box[3][1])**2)
			size.append(w * h)
			if size[i] > size_max:
				size_max = size[i]
				index = i
				xc = x_mid
				yc = y_mid
		key = cv2.waitKey(10)
		if key == ord('q'):
			break
	capture.release()
	cv2.destroyAllWindows()

def command_callback(data):
	global xc_array
	global yc_array
	global xarray
	global yarray
	global index

	if index < 20:
		xc_array[index] = xc
		yc_array[index] = yc
		xarray[index] = 180 + index * 5
		yarray[index] = 200 - index * 10
		print("%d/20,pose x,y: %d,%d. cam x,y: %d,%d" % (index+1, xarray[index], 
        yarray[index], xc, yc))
		# reshape to 2D array for linear regression
		xc_array = xc_array.reshape(-1,1)
		yc_array = yc_array.reshape(-1,1)
		xarray = xarray.reshape(-1,1)
		yarray = yarray.reshape(-1,1)
		index = index + 1
	if index == 20:
		Reg_x_yc = LinearRegression().fit(yc_array, xarray)
		Reg_y_xc = LinearRegression().fit(xc_array, yarray)
		k1 = Reg_x_yc.coef_[0][0]
		b1 = Reg_x_yc.intercept_[0]
		k2 = Reg_y_xc.coef_[0][0]
		b2 = Reg_y_xc.intercept_[0]
		s = '' + str(k1) + ' ' + str(b1) + ' ' + str(k2) + ' ' + str(b2) + '\n'
		filename = os.environ['HOME'] + "/thefile.txt"
		file_pix = open(filename, 'w')
		file_pix.write(s)
		file_pix.close()
		print(filename)
		print("Linear Regression for x and yc is :  x = %.5fyc + (%.5f)" % (k1, b1))
		print("Linear Regression for y and xc is :  y = %.5fxc + (%.5f)" % (k2, b2))
		index = 0


def main():
	rospy.init_node('image_converter', anonymous=True)
	threat1 = threading.Thread(target=image_callback)
	threat1.setDaemon(True)
	threat1.start()
	sub2 = rospy.Subscriber("cali_pix_topic", status, command_callback)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
