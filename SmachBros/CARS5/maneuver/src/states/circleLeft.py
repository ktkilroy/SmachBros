#!/usr/bin/env python

import rospy
import smach
import time
import math
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from drive import drive

'''
Class Description:
This state is the CircleLeft state. We pass in the circleLeft_input as the input key containing the data recieved from the Command. We then use this param to pass
in the appropriate radius to our circle left method, which calculates the turning angle and uses this as in input to the drive command. The drive class 
constructs the ackermann message given a speed,angle,duration, and publisher.
'''

class CircleLeft(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['command'],
					   input_keys=['circleLeft_input']	)

	def execute(self, userdata):
		time.sleep(0)
		rospy.loginfo("Executing State: Circle")
		#Set up a topic to publish on
		#drive topic;
		
		drive_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")

		myDrivePublisher = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10 )

		radius = userdata.circleLeft_input
		
		circleLeft(radius, myDrivePublisher)

		return 'command'


def getTurningAngle(radius):
	angle = math.atan(2*math.tan(math.asin(.3/(2*radius)))) #in radians
	return angle

#thus function/equation is not exact, just an OK approximation
#needs to be replaced by a better approximation
def revolutionTime(radius):
	time = (2* math.pi*radius)/ .86125
	return time

#since max steering angle is .34 radians, the max radius is 2.99
# and min radius is .8612
def circleLeft(radius, myPublisher):
	
	speed = 1 #meters / second
	angle = getTurningAngle(radius) #radians 
	angular_velocity = 1
	duration = rospy.Duration(revolutionTime(radius))

	drive(speed, angle, duration, myPublisher)

