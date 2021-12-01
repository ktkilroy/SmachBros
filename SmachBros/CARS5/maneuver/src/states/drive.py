#!/usr/bin/env python

import rospy
import smach
import time
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
)
from tf.transformations import quaternion_from_euler

'''
Class Description:
The drive class is responsible for publishing the AckermandDrive topic to the passed in publisher. The topic published will drive the car at the speed, angle, and duration passed in from the transition state.
'''
def drive(speed, angle, duration, myPublisher):

	rate = rospy.Rate(20) #10hz
	#duration = rospy.Duration(1.0) #duration in seconds
	start = rospy.Time.now()

	#construct the Ackermann Message
	ackermannMsg = AckermannDrive(steering_angle=angle, speed=speed)

	while rospy.Time.now() - start < duration:
		#publish velocity and steering_angle
		myPublisher.publish(AckermannDriveStamped(drive = ackermannMsg))
		rate.sleep()
