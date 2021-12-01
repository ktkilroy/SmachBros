import rospy
import math
import numpy as np
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry
from math import sqrt

pose = []

#since max steering angle is .34 radians, the max radius is 2.99
# and min radius is .8612
def circleLeft(radius, myPublisher):
	
	speed = 1 #meters / second
	angle = getTurningAngle(radius) #radians 
	angular_velocity = 1
	duration = rospy.Duration(revolutionTime(radius))

	drive(speed, angle, duration, myPublisher)

def getPosition():

	pose_topic = "/car/vesc/odom"
	odomMsg = rospy.wait_for_message(pose_topic, Odometry)
	
	x_position = odomMsg.pose.pose.position.x
	y_position = odomMsg.pose.pose.position.y
	z_position = odomMsg.pose.pose.position.z
	
	position = [x_position, y_position, z_position]

	return position
	
def getTurningAngle(radius):
	angle = math.atan(2*math.tan(math.asin(.3/(2*radius)))) #in radians
	return angle

#thus function/equation is not exact, just an OK approximation
#needs to be replaced by a better approximation
def revolutionTime(radius):
	time = (2.0*math.pi*radius)/ .86125
	return time

def circleRight(radius, myPublisher):
	
	speed = 1 #meters / second
	angle = getTurningAngle(radius) #radians 
	duration = rospy.Duration(revolutionTime(radius))

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
		pose.append(getPosition()) #get position while driving
		rate.sleep()


def getDistance(pos_one, pos_two):
	distance = np.linalg.norm(np.array(pos_one) - np.array(pos_two))
	return distance 


 
# Function to find the circle on
# which the given three points lie
def findCircle(x1, y1, x2, y2, x3, y3) :
    x12 = x1 - x2;
    x13 = x1 - x3;
 
    y12 = y1 - y2;
    y13 = y1 - y3;
 
    y31 = y3 - y1;
    y21 = y2 - y1;
 
    x31 = x3 - x1;
    x21 = x2 - x1;
 
    # x1^2 - x3^2
    sx13 = pow(x1, 2) - pow(x3, 2);
 
    # y1^2 - y3^2
    sy13 = pow(y1, 2) - pow(y3, 2);
 
    sx21 = pow(x2, 2) - pow(x1, 2);
    sy21 = pow(y2, 2) - pow(y1, 2);
 
    f = (((sx13) * (x12) + (sy13) *
          (x12) + (sx21) * (x13) +
          (sy21) * (x13)) // (2 *
          ((y31) * (x12) - (y21) * (x13))));
             
    g = (((sx13) * (y12) + (sy13) * (y12) +
          (sx21) * (y13) + (sy21) * (y13)) //
          (2 * ((x31) * (y12) - (x21) * (y13))));
 
    c = (-pow(x1, 2) - pow(y1, 2) -
         2 * g * x1 - 2 * f * y1);
 
    # eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
    # where centre is (h = -g, k = -f) and
    # radius r as r^2 = h^2 + k^2 - c
    h = -g;
    k = -f;
    sqr_of_r = h * h + k * k - c;
 
    # r is the radius
    r = round(sqrt(sqr_of_r), 5);
 
    print("Centre = (", h, ", ", k, ")");
    print("Radius = ", r);

if __name__ == '__main__':
	#initialize my node
	rospy.init_node("main")	

	#Set up a topics to publish on

	#drive topic;
	drive_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")
	myDrivePublisher = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10 )
	
	pose.append(getPosition()) #get initial position 
	circleLeft(1.0, myDrivePublisher) #drive forward
	rospy.sleep(1) #wait until final position has been published
	pose.append(getPosition()) #get final position
	
	length = len(pose)

	findCircle(pose[2*length/5][0], pose[2*length/5][1], pose[3*length/5][0], pose[3*length/5][1], pose[4*length/5][0], pose[4*length/5][1])

