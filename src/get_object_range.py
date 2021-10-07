#!/usr/bin/env python
#Convert object location and LIDAR scan to range and angular values of the detected object

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import numpy as np

###################################
## VARIABLE DECLARATION AND SETUP
###################################

cameraHorizontalFOV = 62.2          # Field of view (in degrees)
cameraVerticalFOV = 48.8            # Field of view (in degrees)

objectAngleCenter = 88888.0               # Angle of ObjectCenter in camera View (in degrees) 
objectAngleMin = 88888.0                  # Angle of Object leftmost point in camera View (in degrees)
objectAngleMax = 88888.0                  # Angle of Object rightmost point in camera View (in degrees)

imageWidth = 640.0                  # Width of the image

objectRangeMean = 77777.0           # Mean range of the detected object
laserScanLength = 360				# Length of laser scan range array

###################################
## Function Declaration
###################################
def pixelToAngle(pixel):
	angle = (pixel/imageWidth)*cameraHorizontalFOV - (1.0/2.0)*cameraHorizontalFOV
	return angle

def convertToAngles(objectPixel):
	global objectAngleCenter
	global objectAngleMin
	global objectAngleMax

	imageWidth = objectPixel.z
	#Convert object pixel data (The x position of the center of the object, the width of the object, and the width of the image) 
	#as angular values
	if objectPixel.x != 99999.0:
		# objectAngle goes from [-1/2 * cameraHorizontalFOV, +1/2 * cameraHorizontalFOV)
		objectAngleCenter = pixelToAngle(objectPixel.x)
		objectAngleMin_test = pixelToAngle(objectPixel.x - (1.0/2.0)*objectPixel.y)
		objectAngleMax_test = pixelToAngle(objectPixel.x + (1.0/2.0)*objectPixel.y)

		# Flip the rotational coordiante system for the LIDAR
		objectAngleMax = -objectAngleMin_test
		objectAngleMin = -objectAngleMax_test

		# print("AngleMin: ",objectAngleMin%laserScanLength)
		# print("AngleCenter: ",objectAngleCenter)
		# print("AngleMax: ",objectAngleMax%laserScanLength)
	else:
		objectAngleCenter = 88888.0
		objectAngleMin = 88888.0
		objectAngleMax = 88888.0



def computeObjectRange(laserScanObject):
	global objectRangeMean
	#compute the object distance from laser scan values
	#print("laserScanReceived")
	#print("ObjAngleCenter in laser callback: ",objectAngleCenter)
	laserScanObjectRanges = list(laserScanObject.ranges)
	#print("Seq: ",laserScanObject.header.seq)
	#print(laserScanObjectRanges)
	if objectAngleCenter != 88888.0:
		indexMin = int(np.floor(objectAngleMin))%laserScanLength
		indexMax = (int(np.ceil(objectAngleMax))+1)%laserScanLength
		print("Index Min: ",indexMin)
		print("Index Max: ",indexMax)

		if(indexMin>indexMax):
			object_Ranges = laserScanObjectRanges[indexMin:] + laserScanObjectRanges[:indexMax]
		else:
			object_Ranges = laserScanObjectRanges[indexMin:indexMax]
			
		object_Ranges_nonzero = [v for v in object_Ranges if v != 0.0]
		#print(object_Ranges_nonzero)
		#print("ObjectRanges: ",object_Ranges)

		if len(object_Ranges_nonzero) != 0:
			objectRangeMean = sum(object_Ranges_nonzero)/len(object_Ranges_nonzero)
		else:
			objectRangeMean = 77777.0
		print("objectRangeMean: ",objectRangeMean)
	else:
		objectRangeMean = 77777.0

	location_Object = Point(objectAngleCenter, objectRangeMean, 66666.0)
	pub.publish(location_Object)

	#print("********************************************************************************************************************************************************")


def Init():
	global pub

	rospy.init_node('get_object_range', anonymous=True)
	# Subscribe to object location topic /imageLocation
	rospy.Subscriber("/imageLocation", Point, convertToAngles, queue_size=1)

	# Subscribe to LIDAR scan topic /scan
	rospy.Subscriber("/scan", LaserScan, computeObjectRange, queue_size=1)

	# Publish angle and distance
	pub = rospy.Publisher("/object_Location", Point, queue_size=10)

	rospy.spin()


if __name__ == '__main__':
	try:
		Init()
	except rospy.ROSInterruptException:
		pass