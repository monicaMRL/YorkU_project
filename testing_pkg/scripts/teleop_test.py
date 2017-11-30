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

ANGULAR_CHANGE = 0.2

throttleValue = 0.0
headingAngle = 0.0

thorttleMax = 0.5
throttleMin = 0.0
angleMax = 1.0
angleMin = -1.0


#------------------- Helper functions -------------------------------------------------------

def joystickHandler(data):
	global headingAngle, throttleValue
	
	# Set Heading of the boat.	
	if (data.axes[0] == 1 and headingAngle > angleMin):
		headingAngle -= ANGULAR_CHANGE

	if (data.axes[0] == -1 and headingAngle < angleMax):
		headingAngle += ANGULAR_CHANGE

	# Set velocity for boat
	if (data.axes[1] == 1 and throttleValue < thorttleMax):
		throttleValue += THRUST
	#elif (data.axes[1] == 0 and throttleValue > throttleMin):
		#throttleValue -= FRICTION

	if (data.axes[1] == -1 and throttleValue > throttleMin):
		throttleValue -= FRICTION
	

def statusCallback(data):
	#print data
	pass



#------------------- main --------------------------------------------------------------------
if __name__ == "__main__":
	rospy.init_node('joystickControl')

	#publishers
	throttlePub = rospy.Publisher('setThrottle', Float32, queue_size=10)
	#headingPub = rospy.Publisher('setRudder', Float32, queue_size=10)
	#headingPub = rospy.Publisher('commandHeading', Float32, queue_size=10)

	#subscribers
	rospy.Subscriber('vesselStatus', Boat, statusCallback)
	rospy.Subscriber('joy', Joy, joystickHandler)

	rate = rospy.Rate(10) # 10hz	

	while not rospy.is_shutdown():
		is_danger = rospy.get_param("heartbeat_lost")
		if is_danger:
			throttleValue = 0.0
			throttlePub.publish(Float32(throttleValue))
		elif not is_danger:			
			throttlePub.publish(throttleValue)
		
		#headingPub.publish(headingAngle)
		rate.sleep()	



















