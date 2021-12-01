#!/usr/bin/env python

import rospy 
import smach 
import time 	
from Queue import Queue
import json

'''
Class Description:
This is the main state of the State Machine, which is responsible for delegating all tasks to the appropriate state transitions. 
After reading in the json data and parsing the information for transition states and parameters, in the execute we continue to send the parameters with the corresponding transition.
'''


#planning state
class Command(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['driveForward', 'reverse', 			'turnRight', 'turnLeft','threePointTurn','circleRight', 'circleLeft','stop','exit'], 
					  output_keys=['command_out'])

		global transitionsQueue 
		global parametersQueue
		
		transitionsQueue  = Queue(maxsize = 10)
		parametersQueue  = Queue(maxsize = 10)
		with open('task_plan.json') as file:
			plan = json.load(file)


		for i in plan['maneuvers']:
			transitionsQueue.put(i['transition'])
			parametersQueue.put(i['parameter'])

	def execute(self, userdata):
		time.sleep(0)
		rospy.loginfo("Executing State: Command")
	
		if not parametersQueue.empty():
			userdata.command_out = float( parametersQueue.get() )

		if  transitionsQueue.empty(): 
			return 'exit'
		else:
			transition = transitionsQueue.get()  
			return transition



