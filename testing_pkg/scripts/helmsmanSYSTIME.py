#!/usr/bin/env python

#'''
#Publisher: Publishes to setRudder
#		--> Subscriber to setRudder: Arduino node

#Subscriber: Subscribes to vesselStatus
#		--> Publisher of vesselStatus: Arduino node
#	    Subscribes to commandHeading
#		--> Publisher to commandHeading: 

#sets the rudder value by using PID controller.
#'''
#---------------------------------------------------imports------------------------------------------------------------
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from minnow.msg import Boat
from PID import PID

#---------------------------------------------------global variables-------------------------------------------------------

currentHeading = 0
desiredHeading = 0
pid = PID()
pid.SetKp(0.5/90.0)

#HEARTBEAT_MISS = 0
NEW_HEARTBEAT = 0
SYS_TIME = 0
flag = 0

#------------------------------------------------------functions---------------------------------------------------
def heartbeatCallback(heartbeat):
#'''
#	sets new heart beat to arriving data
	
#	@param --> heartbeat msg
#'''	
	global NEW_HEARTBEAT, flag
	
	NEW_HEARTBEAT = heartbeat.data
	print "heartbeat time",NEW_HEARTBEAT
	
	if flag == 0:
		rospy.Subscriber('vesselStatus', Boat, statusCallback)
		rospy.Subscriber('commandHeading', Float32, headingCallback)
		flag = 1
			

def resetCallback(data):
	global SYS_TIME
	if data.data == 'reset':
		SYS_TIME = rospy.get_time()
		heartbeat_guard.publish(String("Up again"))


def statusCallback(data):
	global currentHeading
	global desiredHeading
	global pub, SYS_TIME

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

	print 'Subraction',SYS_TIME - NEW_HEARTBEAT 
	if(SYS_TIME - NEW_HEARTBEAT >= 20 or SYS_TIME - NEW_HEARTBEAT < 0):
		pub.publish(Float32(0.0))
		SYS_TIME = 0
		heartbeat_guard.publish(String("Heartbeat lost, last beat time: %s"%(String(SYS_TIME))))
		print "MISS"
	elif(SYS_TIME - NEW_HEARTBEAT < 20 and SYS_TIME - NEW_HEARTBEAT >= 0):
		pub.publish(Float32(v))
		#heartbeat_guard.publish(String("O.K"))
		print "OK"
	else:
		print "Something went wrong"	
	#print "Current heading "  + str(currentHeading) + " Desired heading "  + str(desiredHeading) + " Ouptut angle is " + str(v)

def headingCallback(data):
	global desiredHeading
	global pid
	desiredHeading = data.data
	print "heading updated to " + str(data.data)
	pid.Initialize()

#---------------------------------------------------- Main -------------------------------------------------------

if __name__ == '__main__':
	pid.Initialize()
	try:
		rospy.init_node('helmsman')
		SYS_TIME = rospy.get_time()
		print "system time",SYS_TIME
		
		pub = rospy.Publisher("setRudder", Float32)
		pub.publish(Float32(0.0))
		heartbeat_guard = rospy.Publisher("systemStatus", String)
		
		rospy.Subscriber('heartBeat', Float32, heartbeatCallback)
		rospy.Subscriber('resetSystem', String, resetCallback)
		
		rospy.spin()
	except rospy.ROSINterrupteException: pass
