#!/usr/bin/env python

import rospy
import smach
import time
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from drive import drive

'''
Class Description:
Here we rotate the opposite direction of turn left, so we simply take the negative of .34.
'''

class TurnRight(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['command'])

	def execute(self, userdata):
		time.sleep(0)
		rospy.loginfo("Executing State: TurnRight")
		#Set up a topic to publish on
		#drive topic;
		
		drive_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")

		myDrivePublisher = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10 )
		
		turnRight(myDrivePublisher)
		
		return 'command'

def turnRight(myPublisher):
	'''
	function to turn right at a 90 degree angle
	'''
	
	speed = 1
	angle = -.34
	duration = rospy.Duration(1.5708)
	drive(speed, angle, duration, myPublisher)
