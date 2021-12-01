import rospy
import math
import numpy as np
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def turnLeft(myPublisher):
	'''
	function to turn right at a 90 degree angle
	'''
	
	speed = 1
	angle = .34
	duration = rospy.Duration(2.0*math.pi/4)
	drive(speed, angle, duration, myPublisher)


def drive(speed, angle, duration, myPublisher):

	rate = rospy.Rate(100) #10hz
	start = rospy.Time.now()

	#construct the Ackermann Message
	ackermannMsg = AckermannDrive(steering_angle=angle, speed=speed)


	while rospy.Time.now() - start < duration:

		#publish velocity and steering_angle
		myPublisher.publish(AckermannDriveStamped(drive = ackermannMsg))
		rate.sleep()


def getPose():

	pose_topic = "/car/vesc/odom"
	odomMsg = rospy.wait_for_message(pose_topic, Odometry)
	
	position_msg = odomMsg.pose.pose.position 
	orientation_q = odomMsg.pose.pose.orientation 
	
	orientation_q_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

	(roll, pitch , yaw) = euler_from_quaternion(orientation_q_list)
	
	position_list = [position_msg.x, position_msg.y, position_msg.z]
	orientation_list = [roll, pitch, yaw]

	return position_list, orientation_list

if __name__ == '__main__':
	#initialize my node
	rospy.init_node("main")	

	#Set up a topics to publish on

	#drive topic;
	drive_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")
	myDrivePublisher = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10 )

	pose = [] #list to hold position and orientation lists
	pose.append(getPose()) #obtain initial position before maneuvers from odom 
	turnLeft(myDrivePublisher)
	rospy.sleep(1) #wait until positions have been published to odom topic
	pose.append(getPose()) #obtain final position after maneuvers from odom topic

	print("Initial Position :", pose[0][0])
	print("Initial Orientation :", pose[0][1])

	print("Final Position :", pose[1][0])
	print("Final Orientation :", pose[1][1])
