# Project 1 - Basic Maneuvers

## Table of contents
1. [System Requirements](#system-requirements)
2. [Design Definitions](#design-definitions)
3. [DSL](#dsl)
4. [Trace Matrix 1](#trace-matrix-1)
5. [Trace Matrix 2](#trace-matrix-2)
6. [Testing Strategy](#testing-strategy)


## System Requirements
**System Requirement (R1)**: The Jetson Naon Car ("Yoshi") shall conduct a series of maneuvers in a sequence specified in a json task plan provided by the user. The car supports the following operations : circle left, circle right, drive forward, reverse, stop, three point turn, turn left and turn right.   
## Design Definitions
**Definition (D1)**: A SMACH architecture shall be implemented to create a state machine that will support any sequence of maneuvers in the order specified in the task plan.   

**Definition (D2)**: Each possible maneuver shall be represented by a state.   

**Definition (D3)**: The Command state shall be the main node responsible for delegating tasks to other transitions. The Command state thus maps all user requested transitions to the appropriate output states. The Command begins by reading in a json file filled with specific tasks requested from the client. The state loads the file and reads in the desired maneuver requests. The class then places the transitions and the parameters into two separate queues, and returns the transition executable until the transition queue is empty and returns state exit when terminated. In order to deal with the client parameters, the Command continues to return the transition along with the attribute "userdata" as the float of the parameter, which is then read in on the state side to process the specified request. More specifically, the state reads in the parameter sent from the Command (if applicable), and uses the user_data as input for the task. The state then sends the appropriate data in parameter form to a method call of drive, which takes in the modified speed, angle, duration, and publisher information.


**Definition (D4)**: The Drive state will be responsible for recieving the appropriate data in parameter form to publish an AckermannDriveStamped Message with a speed, angle, duration and publisher to publish an AckerMannDriveStamped Message which is the drive command for robots with Ackermann steering.

## DSL
Our Domain-Specific Language (DSL) is a simple json file with a single array whose elements specifies the different maneuvers to be executed by "Yoshi" in the same sequental order as the position in the array. Each element of the array is a two-element dictionary whose first entry is the name of the maneuver to be performed and second entry is a string parameters the maneuver state may need. The general template for this dictionary is as follows: {"transition": "<maneuver>", "parameter": "<parameter>"}. A json file was chosen as our project DSL due to it's unambigous format structure and abundant support in python libraries.   
	
**It is important to note that the DSL parser requires that each maneuver must contain a parameter.**

Here is an example of a possible json client task:
```json
{
	"maneuvers": [
		{
		"transition":"driveForward",
		"parameter": "2.0"
		},
		{
		"transition":"reverse",
		"parameter": "2.5"
		},
		{
		"transition":"stop",
		"parameter": "2.0"
		},
		{
		"transition":"turnLeft",
		"parameter": "0.0"
		}
	]

}
```

## Trace Matrix 1
Requirements to Design Specifications:

| System Requirements | Design Definition |
|----|----|
| R1 | D1 |
| R1 | D2 |
| R1 | D3 |
| R1 | D4 |


## Trace Matrix 2
Design Specifications to Code Files:

| Design Definitions | Code Files           |
|----|-------------------|
| D1 | [main.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/main.py)           |
| D2 | [circleLeft.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/states/circleLeft.py)     |
| D2 | [circleRight.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/states/circleRight.py)    |
| D2 | [driveForward.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/states/driveForward.py)   |
| D2 | [reverse.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/states/reverse.py)       |
| D2 | [stop.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/states/stop.py)           |
| D2 | [threePointTurn.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/states/threePointTurn.py) |
| D2 | [turnLeft.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/states/turnLeft.py)       |
| D2 | [turnRight.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/states/turnRight.py)      |
| D3 | [command.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/states/command.py)        |
| D4 | [drive.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/states/drive.py)          |



## Testing Strategy

### Unit Testing

Each car maneuver was thoroughly tested in isolation utilizing both visualization and quantitative techniques. For the visualization component, RVIZ, a powerful ROS 3D visualization tool was utilized to simulate a virtual Mushr car in a general environment where maneuvers can be tested. Using this tool, our team was able to generally inspect each maneuver and visually confirm they conformed to general expectations. This simulator was used in conjuction with user-defined testing functions that logged the position and orientation (each a 3-dimensional vector) of the car before and after each maneuver to verify that the position and orientation of the car quantitatively conformed to expectations. This also provided our team a method to quantify the margin of error for each maneuver that may potentially be used for safety requirement specifications in the future. This testing function obtained the quantities of interest by obtaining a single pose message from the topic "/car/vesc/odom" of type Odometry. More details can be found in the code snippet below.   

```python
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
```
  
Each car maneuver was isolated by creating separate scripts, which can be found in the [unit_testing](https://github.com/SAREC-Lab/CARS5/tree/main/maneuver/src/unit_testing) folder under the following file names: [driveForwardTest.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/unit_testing/driveForwardTest.py), [reverseTest.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/unit_testing/reverseTest.py), [turnRighTest.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/unit_testing/turnRighTest.py), [turnLeftTest.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/unit_testing/turnLeftTest.py), [circleTest.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/unit_testing/circleTest.py) and [threePointTurnTest.py](https://github.com/SAREC-Lab/CARS5/blob/main/maneuver/src/unit_testing/threePointTurnTest.py). Each maneuver was then independently tested numerous times with different input parameters (i.e. driveForward(2), driveForward(5), driveForward(8)) to quantitatively ascertain the precision of each maneuver by averaging the error over all trials. A video of a test trial for each maneuver can be found in the [Appendix B: Testing Videos](#appendix-b:-testing-videos) section below. 

Each testing script was generally formatted as shown in the snippet below, where the getPose() function is called whose returned value is appended to a list prior to a specific maneuver. The rospy.sleep(1) function is called in order to provide ROS with enough time to publish the final pose message onto the "/car/vesc/odom" topic. Then, the getPose() function is once again called. 

```python
if __name__ == '__main__':
	#initialize my node
	rospy.init_node("main")	

	#Set up a topics to publish on

	#drive topic;
	drive_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")
	myDrivePublisher = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10 )

	pose.append(getPose()) #obtain initial position before maneuvers from odom 
	threePointTurn(myDrivePublisher)
	rospy.sleep(1) #wait until positions have been published to odom topic
	pose.append(getPose()) #obtain final position after maneuvers from odom topic

	
	
	print("Initial Position :", pose[0][0])
	print("Initial Orientation :", pose[0][1])

	print("Final Position :", pose[1][0])
	print("Final Orientation :", pose[1][1])
```


For unit testing of the circleLeft and circleRight maneuvers, the unit testing script circleTest.py uses arithmetic formulas to ascertain the center and radius of the circle given 3 points on which the circle passes through. These points are similarly gathered by utilizing a slight variation of the getPose() function. Three (x,y) coordinates are then passed onto the function below to calculate the radius which is then compared to the input of the maneuvers. 


```python
def findCircle(x1, y1, x2, y2, x3, y3) :
	x12 = x1 - x2;
	
	y12 = y1 - y2;
	#repeat to calculate (x13,y13), (x21,y21) and (x31,y31)
	
	# the distance between x1 and x3 using x1^2 - x3^2
	sx13 = pow(x1, 2) - pow(x3, 2);
	# y1^2 - y3^2
	sy13 = pow(y1, 2) - pow(y3, 2);
	
	#repeat for |x2,x1| and |y2,y1|
	#calculate the center of the circle (f,g) and the c value for the
	# eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
	# where centre is (h = -g, k = -f) and
	# radius r as r^2 = h^2 + k^2 - c
	h = -g;
	k = -f;
	sqr_of_r = h * h + k * k - c;
	 
	# r is the radius
	r = round(sqrt(sqr_of_r), 5);
```
 ### Integration Testing
 
 A similar approach was undertaken for systems integration testing of the maneuvers and state machine. To test this integration, the state machine was executed numerous times, each with different task plans specified in the DSL. The maneuvers were then inspected visually in the simulator; aditionally, the log output to the console of each state machine was cross-referenced with the DSL to confirm the task plan was followed accurately and without errors. An example of one integration test can be found in Appendix B. 
 
 ### Appendix A: How Each Maneuver Works

Each maneuver utilized a single wrapper function named drive() that accepts speed, angle and duration parameters, creates an AckermannDrive message and publishes to a specified topics for a period of time dictated by the duration parameter. In essence, each maneuver is fundamentally either a drive() function with distinct parameters (i.e. reverse, turnLeft, circle) or a composition of the drive() function (i.e. ThreePointTurn). the code snippet for the drive function is showed below.

```python
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
```

### Appendix B: Testing Videos

Video displays the systems integration test involving the execution of the below task plan in a state machine. 
Task Plan

```
Drive Forward 2 meters
Reverse 3 meters
Stop for 1 second
Turn Left
Three Point Turn
Circle Left with radius of 1 meter
Circle Right with radius of 2 meters
Turn Right
```

https://user-images.githubusercontent.com/37822940/135550636-dd123c68-f7e4-4c4f-ba89-286a5f3f6c77.mp4

Video display a three point turn maneuver unit test

https://user-images.githubusercontent.com/37822940/135550093-16cf7d14-4ad3-470a-9056-2c78946458b1.mp4


Video displays a drive 4 meters forward unit test

https://user-images.githubusercontent.com/37822940/135374164-f3779bfa-405e-4deb-9608-45f8ce0af116.mp4

Video displays a reverse 4 meters maneuver unit test

https://user-images.githubusercontent.com/37822940/135549822-a647c015-8ba4-4dee-9db5-70a6ea99f9ad.mp4

Video displays a left circle (radius = 1 meter) maneuver unit test  

https://user-images.githubusercontent.com/37822940/135550049-99e60693-c772-440e-b4f2-d5b8a22b5df9.mp4

Video displays a turn left maneuver unit test

https://user-images.githubusercontent.com/37822940/135550106-789fdf06-cdb0-4c33-a348-0c50d848e0a2.mp4


Video displays a turn right maneuver unit test

https://user-images.githubusercontent.com/37822940/135550118-30247122-4be4-4c90-bf46-62966bd0250f.mp4
						  
						  
## Comments and Grades
Pretty good job, it is recommended to use rosrun for the ROS projects as discussed in class. You should have a branch named project1.
** Grade:100/100 **
						  
						  
						  
						 

