#!/usr/bin/env python
# move Robot based on object Location

import rospy
from geometry_msgs.msg import Point,Twist
import numpy as np

def move_Robot(location_Object):
	# first reduce the rotational error, then reduce the translational error
	move_cmd = Twist()
	k_Rotation = -0.015
	k_Translation = -0.3

	error_Rotation = location_Object.x
	error_Translation = 0.5 - location_Object.y
	if location_Object.x != 88888.0:
		if (abs(error_Rotation) >= 2.5) and (location_Object.y != 77777.0):
			move_cmd.angular.z = np.sign(k_Rotation*error_Rotation)* np.clip(np.abs(k_Rotation*error_Rotation),0.0,0.2)
		elif abs(error_Translation) >= 0.1 and location_Object.y <= 2.5:
			move_cmd.linear.x = np.sign(k_Translation*error_Translation)* np.clip(np.abs(k_Translation*error_Translation),0.03,0.1)

	pub.publish(move_cmd)

def Init():
	global pub
	
	rospy.init_node('move_Robot', anonymous=True)
	# Subscribe to the /object_Location topic
	rospy.Subscriber('/object_Location', Point, move_Robot, queue_size=1)
	#Publish to cmd_vel topic
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	rospy.spin()



###################################
## MAIN
###################################

if __name__ == '__main__':
	try:
		Init()
	except rospy.ROSInterruptException:
		pass