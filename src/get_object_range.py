#!/usr/bin/env python
#Convert object location and LIDAR scan to range and angular values of the detected object

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

###################################
## VARIABLE DECLARATION AND SETUP
###################################

cameraHorizontalFOV = 62.2          # Field of view (in degrees)
cameraVerticalFOV = 48.8            # Field of view (in degrees)

objectAngleCenter = 88888.0               # Angle of ObjectCenter in camera View (in degrees) 
objectAngleMin = 88888.0                  # Angle of Object leftmost point in camera View (in degrees)
objectAngleMax = 88888.0                  # Angle of Object rightmost point in camera View (in degrees)

imageWidth = 640.0                  # Width of the image

###################################
## Function Declaration
###################################
def pixelToAngle(pixel):
	angle = (pixel/imageWidth)*cameraHorizontalFOV - (1.0/2.0)*cameraHorizontalFOV
	return angle

def convertToAngles(objectPixel):
	global objectAngle
	global objectWidth
	global imageWidth

	imageWidth = objectPixel.z
	#Convert object pixel data (The x position of the center of the object, the width of the object, and the width of the image) 
	#as angular values
	if objectPixel.x != 99999.0:
		# objectAngle goes from [-1/2 * cameraHorizontalFOV, +1/2 * cameraHorizontalFOV)
		objectAngleCenter = pixelToAngle(objectPixel.x)
		objectAngleMin = pixelToAngle(objectPixel.x - (1.0/2.0)*objectPixel.y)
		objectAngleMax = pixelToAngle(objectPixel.x + (1.0/2.0)*objectPixel.y)
		print("AngleMin: ",objectAngleMin)
		print("AngleCenter: ",objectAngleCenter)
		print("AngleMax: ",objectAngleMax)
	else:
		objectAngleCenter = 88888.0
		objectAngleMin = 88888.0
		objectAngleMax = 88888.0

def computeObjectRange(laserScanObject):
	#compute the object distance from laser scan values
	pass

def Init():
	rospy.init_node('get_object_range', anonymous=True)
	# Subscribe to object location topic /imageLocation
	rospy.Subscriber("/imageLocation", Point, convertToAngles, queue_size=1)

	#Subscribe to LIDAR scan topic /scan
	rospy.Subscriber("/scan", LaserScan, computeObjectRange, queue_size=1)

	rospy.spin()


if __name__ == '__main__':
	try:
		Init()
	except rospy.ROSInterruptException:
		pass