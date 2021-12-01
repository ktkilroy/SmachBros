#!/usr/bin/env python

import rospy
import smach
import time
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from drive import drive

def driveForward(distance, myPublisher):
	'''
	function to drive foward at speed of 1 meter per second. 
	'''
	duration = rospy.Duration(distance)
	speed = 1
	angle = 0.0
	
	drive(speed, angle, duration, myPublisher)

'''
Class Description:
The DriveForward class takes in the requested amount of distance to drive forward, and sends the drive publisher with the drive topic to the drive class, along with the drive_input.
'''
#DriveForward State
class DriveForward(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['command'],
					   input_keys=['drive_input'])


	def execute(self, userdata):
		time.sleep(2)
		rospy.loginfo("Executing State: Drive Forward")
		distance = userdata.drive_input
		#Set up a topic to publish on
		#drive topic;
		
		drive_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")

		myDrivePublisher = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10 )
		driveForward(distance, myDrivePublisher)
		
		return 'command'
