#!/usr/bin/env python

import rospy
import smach
import time
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from drive import drive

'''
Class Description:
For this class, we set the speed to 0 and send this request to drive.
'''

class Stop(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['command'],
					   input_keys=['stop_input'])

	def execute(self, userdata):
		time.sleep(0)
		rospy.loginfo("Executing State: Stop")
		#Set up a topic to publish on
		#drive topic;
		
		drive_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")

		myDrivePublisher = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10 )
		t = userdata.stop_input
		stop(t, myDrivePublisher)
		
		return 'command'


def stop(time, myPublisher):

	duration = rospy.Duration(time)
	speed = 0.0
	angle = 0.0
	
	drive(speed, angle, duration, myPublisher)
