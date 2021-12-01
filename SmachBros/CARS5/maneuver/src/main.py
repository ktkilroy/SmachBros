#!/usr/bin/env python

import rospy
import smach
import time
import json
from Queue import Queue
from states.command import Command
from states.driveForward import DriveForward
from states.reverse import Reverse
from states.turnLeft import TurnLeft
from states.turnRight import TurnRight
from states.threePointTurn import ThreePointTurn
from states.circleRight import CircleRight
from states.circleLeft import CircleLeft
from states.stop import Stop
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
Brief Description of Class:
This class is the main state machine. We open our container and add our main Command state which adds all possible States as transitions. 
We then use the remapping argument as the remapping field maps the in/output_key of a state to a userdata field of the state machine. For example,
calling userdata.command_out will send the appropriate data field to the next transition specified. For DRIVE_FOWARD, setting the attribute of user_data.command_out will 
access the requested parameter input from the Command and input the field directly into its methods.
'''
def main():
	rospy.init_node('main') #initialize our node


	
	sm = smach.StateMachine(outcomes=['exit']) 


	with sm: #opens sm container
		
		#add states to container
		#template smach.StateMachine.add('STATE_NAME', ClassName(), 					transitions={'transition': 'STATE_NAME'})

		#must create a mapping from transition to another state or end node

		smach.StateMachine.add('COMMAND', Command(),
			transitions={'driveForward': 'DRIVE_FORWARD',
				     'reverse': 'REVERSE',
				     'turnLeft': 'TURN_LEFT',
				     'turnRight': 'TURN_RIGHT',
				     'threePointTurn': 'THREE_POINT_TURN',
				     'circleRight': 'CIRCLE_RIGHT',
				     'circleLeft' : 'CIRCLE_LEFT',
				     'stop': 'STOP',
				     'exit':'exit'},
			remapping={'command_out': 'sm_data'})

		smach.StateMachine.add('DRIVE_FORWARD', DriveForward(),
			transitions={'command':'COMMAND'},
			remapping={'drive_input': 'sm_data'})

		smach.StateMachine.add('REVERSE', Reverse(),
			transitions={'command':'COMMAND'},
			remapping={'reverse_input': 'sm_data'})

		smach.StateMachine.add('TURN_LEFT', TurnLeft(),
			transitions={'command':'COMMAND'})

		smach.StateMachine.add('TURN_RIGHT', TurnRight(),
			transitions={'command':'COMMAND'})

		smach.StateMachine.add('THREE_POINT_TURN', ThreePointTurn(),
			transitions={'command':'COMMAND'})

		smach.StateMachine.add('CIRCLE_LEFT', CircleLeft(),
			transitions={'command':'COMMAND'},
			remapping={'circleLeft_input': 'sm_data'})

		smach.StateMachine.add('CIRCLE_RIGHT', CircleRight(),
			transitions={'command':'COMMAND'},
			remapping={'circleRight_input': 'sm_data'})

		smach.StateMachine.add('STOP', Stop(),
			transitions={'command':'COMMAND'},
			remapping={'stop_input': 'sm_data'})

	outcome = sm.execute()


if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass

