#!/usr/bin/env python

"""
	Subscriber: Subscribe to topic joy
			--> publisher of joy, joy node
		    Subscribe to vesselStatus
			--> publisher of vesselStatus, vessel_status_throttled
	
	Publisher: Publishes to setThrottle
			--> Subscriber of setThrottle, Arduino node
		   publishes to commandHeading
			--> Subscriber of commandHeading, helmsman node
"""

#-------------------Imports ---------------------------------------------------------------------
import rospy
from sensor_msgs.msg import Joy
from minnow.msg import Boat
from std_msgs.msg import Float32

#------------------ global variables --------------------------------------------------------------
THRUST = 0.02
FRICTION = 0.03
BREAK = 0.05
ANGULAR_CHANGE = 0.4

JOY_KEY = {'up': False,'down': False}

throttleValue = 0.0
headingAngle = 0.0

thorttleMax = 0.5
throttleMin = 0.0
angleMax = 1
angleMin = -1


#------------------- Helper functions -------------------------------------------------------

def joystickHandler(data):
	global JOY_KEY,headingAngle
	
	# Set Heading of the boat.	
	if (data.axes[0] == 1 and headingAngle > angleMin):
		headingAngle -= ANGULAR_CHANGE
		#headingPub.publish(headingAngle)

	if (data.axes[0] == -1 and headingAngle < angleMax):
		headingAngle += ANGULAR_CHANGE
		#headingPub.publish(headingAngle)

	# Set velocity for boat
	if (data.axes[1] == 1):
		JOY_KEY["up"] = True
	elif (data.axes[1] == 0):
		JOY_KEY["up"] = False 

	# Set velocity for boat
	if (data.axes[1] == -1):
		JOY_KEY["down"] = True
	elif (data.axes[1] == 0):
		JOY_KEY["down"] = False 
	

def getCommand():
	global throttleValue
	
	# Set velocity for boat
	if (JOY_KEY["up"] == True and throttleValue < thorttleMax):
		throttleValue += THRUST
	elif (JOY_KEY["up"] == False and throttleValue > throttleMin):
		throttleValue -= FRICTION

	if (JOY_KEY["down"] == True and throttleValue > throttleMin):
		throttleValue -= FRICTION
	

def statusCallback(data):
	print data




#------------------- main --------------------------------------------------------------------
if __name__ == "__main__":
	rospy.init_node('joystickControl')

	#publishers
	throttlePub = rospy.Publisher('setThrottle', Float32, queue_size=10)
	headingPub = rospy.Publisher("setRudder", Float32, queue_size=10)

	#subscribers
	rospy.Subscriber('vesselStatus', Boat, statusCallback)
	rospy.Subscriber('joy', Joy, joystickHandler)

	rate = rospy.Rate(10) # 10hz	

	while not rospy.is_shutdown():
		getCommand()
		throttlePub.publish(throttleValue)
		headingPub.publish(headingAngle)
		rate.sleep()	



















