#!/usr/bin/env python

import rospy
import smach
import time
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from drive import drive

'''
Class Description:
For this class, we simply take the reverse input (being the amount of distance requested), and then take the negative speed in order to go the opposite direction.
'''

class Reverse(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['command'],
					   input_keys=['reverse_input'])


	def execute(self, userdata):
		time.sleep(1)
		rospy.loginfo("Executing State: Reverse")
		#Set up a topic to publish on
		#drive topic;
		
		drive_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")

		myDrivePublisher = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10 )
		distance = userdata.reverse_input
		
		reverse(distance, myDrivePublisher)

		return 'command'

def reverse(distance, myPublisher):
	'''
	function to drive in reverse at speed of 1 meter per second. 
	'''
	duration = rospy.Duration(distance)
	speed = -1
	angle = 0.0
	
	drive(speed, angle, duration, myPublisher)
