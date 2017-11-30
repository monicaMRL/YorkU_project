#!/usr/bin/env python

"""
Script description: 
	
"""

##============== Imports ==========================================================
import rospy
from std_msgs.msg import Float32
from minnow.msg import Boat
import simpleguitk as gui
from PIL import Image as IM
import math
from random import randint
import time

##================ Global Variables ================================================
minnowImage = IM.open('/home/monica/panda_work/src/visual_graphics/scripts/boat.png').convert("RGBA")
robotImage = gui.load_image(minnowImage)
backImage = IM.open('/home/monica/panda_work/src/visual_graphics/scripts/water.jpg').convert("RGBA")
pondImage = gui.load_image(backImage)

circle_command = 1
no_of_circle = 1

width = 800
height = 600
Frame = None
txtDesiredHeading = None
txtCurrentHeading = None
txtThrottle = None
txtRudder = None
btnCircle = None
btnJoystickControl = None

IMAGE_CENTER = [45, 45]
IMAGE_SIZE = [90, 90]
IMAGE_RADIUS = 35
ROBOT_IMAGE = None
POND_CENTER = [width/2,height/2]
POND_SIZE = [width,height]
LINE_WIDTH = 1
LINE_COLOR = 'blue'


robotPose = list()		# State space of the robot is X,Y,theta
robotPose = [width/2,height/2,3.14]
robotOrientation = 50
orientationColor = 'red'
robotCommadHeading = 4.71
robotHeading = 0

particles = list()
for i in range(50):
	x = randint(0,width/4)
	y = randint(0,height)
	particles.append([x,y])
for j in range(50):
	x = randint(width- width/4 ,width)
	y = randint(0,height)
	particles.append([x,y])
flow = 1
	


##============ Helper functions ==============================================================

def findPoint(point1,angle):
	"""
	Finds second point for the line given one point and angle.
	"""
	return [int(point1[0] + point1[0] * math.cos(angle)),int(point1[1] + point1[1] * math.sin(angle))]
	
def renderObjects(canvas):
	"""
	Reders various graphic objects on the screen
	"""
	
	#background
	canvas.draw_image( pondImage , POND_CENTER \
                ,  POND_SIZE, POND_CENTER\
                ,  POND_SIZE, 0)
	
	#Blue Direction Lines	
	canvas.draw_line([robotPose[0],robotPose[1]],[0,robotPose[1]],LINE_WIDTH,LINE_COLOR)
	canvas.draw_line([robotPose[0],robotPose[1]],[robotPose[0],0],LINE_WIDTH,LINE_COLOR)
	canvas.draw_line([robotPose[0],robotPose[1]],[width,robotPose[1]],LINE_WIDTH,LINE_COLOR)
	canvas.draw_line([robotPose[0],robotPose[1]],[robotPose[0],height],LINE_WIDTH,LINE_COLOR)
	
	#Floating particles for Animation effects	
	for par in particles:
		canvas.draw_circle(par,2,LINE_WIDTH,'black')
		par[1] += flow
		
		if par[1] >= height:
			par[1] = par[1]%height
	
	#Robots desired and current heading	
	canvas.draw_line([robotPose[0],robotPose[1]],findPoint(robotPose,robotPose[2]),LINE_WIDTH + 1,orientationColor)
	canvas.draw_line([robotPose[0],robotPose[1]],findPoint(robotPose,robotCommadHeading),LINE_WIDTH + 2,"green")

	#Robot
	canvas.draw_image( robotImage , IMAGE_CENTER \
                ,  IMAGE_SIZE, [robotPose[0],robotPose[1]]\
                ,  IMAGE_SIZE, robotPose[2])

	canvas.draw_circle([robotPose[0],robotPose[1]], 100, LINE_WIDTH,LINE_COLOR)


def drawHandler(canvas):
	"""
	Keeps drawing objects on the screen with const. Frequency
	"""
	renderObjects(canvas)

def headingCallback(data):
	"""
	Get current heading
	"""
	global robotPose,robotHeading
	
	txtCurrentHeading.set_text(str(data.data))
	robotHeading = data.data

	guiAngle = (data.data + 180) % 360.0
	angleRadians = guiAngle * math.pi / 180.0
	robotPose[2] = angleRadians

def commandHeadingCallback(data):
	"""
	Sets the command heading for the robot
	"""
	global robotCommadHeading

	txtDesiredHeading.set_text(str(data.data))
	robotCommadHeading = ((data.data + 180) % 360) * math.pi / 180.0
	
def vesselStatusCallback(data):
	"""
	Gets the vessel status from the Ardiuno and displays in text boxes
	"""
	txtRudder.set_text(str(data.rudder))
	txtThrottle.set_text(str(data.throttle))
	

def txtHandler(text):
	pass

def northHandler():
	"""
	Make boat go NORTH
	"""
	pub.publish(Float32(90.0))

def southHandler():
	"""
	Make boat go SOUTH
	"""
	pub.publish(Float32(270.0))

def eastHandler():
	"""
	Make boat go EAST
	"""
	pub.publish(Float32(180.0))

def westHandler():
	"""
	Make boat go WEST
	"""
	pub.publish(Float32(0.0))

def circleStartHandler():
	"""
	Make the boat go in circle.
	"""
	global circle_command

	throttle_pub.publish(Float32(0.1))
	while(not circle_command > no_of_circle):
		pub.publish(Float32(180.0))
		time.sleep(2)
		pub.publish(Float32(270.0))
		time.sleep(2)
		pub.publish(Float32(0.0))
		time.sleep(2)
		pub.publish(Float32(90.0))
		time.sleep(2)
		circle_command += 1
	circle_command = 1

def joyControlHandler():
	"""
	Shift the control of boat to Joystick mode.
	"""
	buttonText = btnJoystickControl.get_text()

	print buttonText

	if buttonText == "ON":
		btnJoystickControl.set_text("OFF")
		rospy.set_param("joystick_control", True)
	
	if buttonText == "OFF":
		btnJoystickControl.set_text("ON")
		rospy.set_param("joystick_control", False)
		

def setupGUI():
	"""
	Setup frame and other control objects for GUI
	"""
	global Frame,txtCurrentHeading,txtDesiredHeading,txtThrottle,txtRudder,btnCircle,btnJoystickControl

	#Frame	
	Frame =  gui.create_frame('Minnow Simulator', width, height)
	Frame.set_draw_handler(drawHandler)

	#========Text Boxes =====================================
	txtCurrentHeading = Frame.add_input("Current Heading",txtHandler,100)
	txtDesiredHeading = Frame.add_input("Desired Heading",txtHandler,100)
	txtThrottle = Frame.add_input("Throttle Value",txtHandler,100)
	txtRudder = Frame.add_input("Rudder Value",txtHandler,100)

	#======= Buttons ==============================================
	btnNorth = Frame.add_button("North",northHandler,50)
	btnSouth = Frame.add_button("South",southHandler,50)
	btnEast = Frame.add_button("East",eastHandler,50)
	btnWest = Frame.add_button("West",westHandler,50)

	lblCircle = Frame.add_label('Move_Circle')
	btnCircleStart = Frame.add_button("start",circleStartHandler,25)

	#====== Joystick Control =====================================================
	lblJoystickControl = Frame.add_label('Joystick_Control')
	btnJoystickControl = Frame.add_button("ON",joyControlHandler,50)



##================= Main =======================================================
if __name__ == '__main__':
	rospy.init_node("Graphics_display")

	#GUI setup
	setupGUI()

	#Subscribers
	rospy.Subscriber('currentHeading',Float32, headingCallback)
	rospy.Subscriber('commandHeading',Float32, commandHeadingCallback)
	rospy.Subscriber('vesselStatus',Boat, vesselStatusCallback)

	#Publishers
	pub = rospy.Publisher('commandHeading',Float32,queue_size=10)
	throttle_pub = rospy.Publisher("setThrottle",Float32)
	rate = rospy.Rate(2)
		
	Frame.start()

	





























