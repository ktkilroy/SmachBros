#!/usr/bin/env python

import rospy
import smach
import time
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from drive import drive
import math

class ThreePointTurn(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['command'])

	def execute(self, userdata):
		time.sleep(0)
		rospy.loginfo("Executing State: Three Point Turn")
		#Set up a topic to publish on
		#drive topic;
		
		drive_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")

		myDrivePublisher = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10 )
		
		threePointTurn(myDrivePublisher)
		
		return 'command'

def threePointTurn(myPublisher):

	duration1 = rospy.Duration((2.0 * math.pi)/4.0)
	#step 1: complete a left turn
	drive(1, .34, duration1, myPublisher)
	#step 2: reverse at a more narrow angle
	duration2 = rospy.Duration((2.0 * math.pi)/5.0)
	drive(-1, -.25, duration2, myPublisher)
	#step 3: drive forward at an angle
	drive(1, .25, duration2, myPublisher)
