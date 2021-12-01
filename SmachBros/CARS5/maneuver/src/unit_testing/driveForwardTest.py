import rospy
import math
import numpy as np
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry

pose = []

def getPosition():

	pose_topic = "/car/vesc/odom"
	odomMsg = rospy.wait_for_message(pose_topic, Odometry)
	
	x_position = odomMsg.pose.pose.position.x
	y_position = odomMsg.pose.pose.position.y
	z_position = odomMsg.pose.pose.position.z
	
	position = [x_position, y_position, z_position]

	return position
	

def driveForward(distance, myPublisher):
	'''
	function to drive foward at speed of 1 meter per second. 
	'''
	duration = rospy.Duration(distance)
	speed = 1
	angle = 0.0
	
	drive(speed, angle, duration, myPublisher)



def drive(speed, angle, duration, myPublisher):

	rate = rospy.Rate(60) #10hz
	#duration = rospy.Duration(1.0) #duration in seconds
	start = rospy.Time.now()

	#construct the Ackermann Message
	ackermannMsg = AckermannDrive(steering_angle=angle, speed=speed)

	while rospy.Time.now() - start < duration:

		#publish velocity and steering_angle
		myPublisher.publish(AckermannDriveStamped(drive = ackermannMsg))
		rate.sleep()


def getDistance(pos_one, pos_two):
	distance = np.linalg.norm(np.array(pos_one) - np.array(pos_two))
	return distance 


if __name__ == '__main__':
	#initialize my node
	rospy.init_node("main")	

	#Set up a topics to publish on

	#drive topic;
	drive_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")
	myDrivePublisher = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10 )
	
	pose.append(getPosition()) #get initial position 
	driveForward(4.0, myDrivePublisher) #drive forward
	rospy.sleep(1) #wait until final position has been published
	pose.append(getPosition()) #get final position


	print("Initial Position: ", pose[0])
	print("Final Position: ", pose[1])
	print("Distance Traveled: ", getDistance(pose[0], pose[1]))


