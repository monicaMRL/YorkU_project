#!/usr/bin/env python

"""
Publisher: Publishes to setRudder
		--> Subscriber to setRudder: Arduino node

Subscriber: Subscribes to vesselStatus
		--> Publisher of vesselStatus: Arduino node
	    Subscribes to commandHeading
		--> Publisher to commandHeading: 

sets the rudder value by using PID controller.
"""

#---------------------------------- Imports ------------------------------------------------------------

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from minnow.msg import Boat
from PID import PID

#------------------------------------ Global variables -------------------------------------------------
currentHeading = 0
desiredHeading = 0
pid = PID()
pid.SetKp(0.5/90.0)

GUARD = 0
HEARTBEAT_TOGGLE = 0
#-------------------------------------- call back functions ------------------------------------------
def statusCallback(data):
	global currentHeading
	global desiredHeading
	global pub
	global HEARTBEAT_TOGGLE, GUARD

	currentHeading = data.yaw
	dt = desiredHeading - currentHeading
	if dt > 180:
		dt = dt - 360
	if dt < - 180:
		dt =  dt + 360
	v = pid.GenOut(dt)
	if v > 1:
		v = 1
	if v < -1:
		v = -1

	if (HEARTBEAT_TOGGLE > -200 and HEARTBEAT_TOGGLE < 400 and GUARD == 0):
		pub.publish(Float32(v))
		HEARTBEAT_TOGGLE -= 1
		print "Safe and counter is %s"%HEARTBEAT_TOGGLE
	elif ((HEARTBEAT_TOGGLE < -200 or HEARTBEAT_TOGGLE > 400) and GUARD == 0):
		#pub.publish(Float32(0.0))
		throttle_pub.publish(Float32(0.0))
		#print "danger and rudder is 0"
		GUARD = 1
	else:
		print "In reset Mode cant change speed"
		#pub.publish(Float32(0.0))
		throttle_pub.publish(Float32(0.0))
		GUARD = 1
	#print "Current heading "  + str(currentHeading) + " Desired heading "  + str(desiredHeading) + " Ouptut angle is " + str(v)

def headingCallback(data):
	global desiredHeading
	global pid
	desiredHeading = data.data
	print "heading updated to " + str(data.data)
	pid.Initialize()

def heartbeatCallback(Heartbeat):
	global HEARTBEAT_TOGGLE
	
	if GUARD == 0:
		HEARTBEAT_TOGGLE += 1


def resetCallback(data):
	global 	HEARTBEAT_TOGGLE, GUARD
	
	HEARTBEAT_TOGGLE = 0
	GUARD = 0


#------------------------------------------ Main ------------------------------------------------------------------
if __name__ == '__main__':
	pid.Initialize()
	rospy.init_node('helmsman')

	# --- Publishers ----------------------
	pub = rospy.Publisher("setRudder", Float32)
	pub.publish(Float32(0.0))
	throttle_pub = rospy.Publisher("setThrottle", Float32)

	#---- Subscribers -------------------------------------
	rospy.Subscriber('vesselStatus', Boat, statusCallback)
	rospy.Subscriber('commandHeading', Float32, headingCallback)
	rospy.Subscriber('heartBeat', String, heartbeatCallback)
	rospy.Subscriber('resetSystem', String, resetCallback)

	rospy.spin()
	
