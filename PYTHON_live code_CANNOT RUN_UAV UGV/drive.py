#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from Adafruit_MotorHAT import Adafruit_MotorHAT
import time
import atexit
from math import pow, atan2, sqrt, pi, degrees
from tf.transformations import euler_from_quaternion

#arm motors
mh = Adafruit_MotorHAT(addr=0x60)
MotorLeft = mh.getMotor(1)
MotorRight = mh.getMotor(2)
ElectroMagnet = mh.getMotor(3)
MotorLeft.run(Adafruit_MotorHAT.BACKWARD)
MotorRight.run(Adafruit_MotorHAT.BACKWARD)
ElectroMagnet.run(Adafruit_MotorHAT.BACKWARD)

#global variables
lasterror=0
targets=[[2,-9],[0,-7],[0,-3],[0,-2],[1,-1],[1,0]] #list of waypoints
##targetIndex = 0 #index first target
targetIndex = 2 ##take out
##mode = 0 #start in wait
mode = 2 ##take out
aeroPose = [1, -11, 0]
#groundPose = [2, -11, 0]
obstaclePose = [0, -5]
##flyMode = 0
flyMode = 2 #take out


def ground(data): #retrieve UGV pose data
	#global groundPose
	groundPose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
	#global groundQuat
	groundQuat = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
	global lasterror
	global targetIndex
	global mode
	#modeSwitcher = data.data
	target = targets[targetIndex]
	    
	if mode == 0: #wait for aero to match position
		aeroDist = round(sqrt(pow((aeroPose[0] - (groundPose[0])), 2) + pow((aeroPose[1] - groundPose[1]), 2)),3)
		if aeroDist < 0.1:	#aero is following
			time.sleep(2)
			mode = 1 #drive
		
	elif mode == 1: #drive to waypoints
		#transform orientation
		[roll, pitch, yaw] = euler_from_quaternion(groundQuat)
		current_theta = round(degrees(yaw),3)
		#desired angle and error 
		steering_angle = round(degrees(atan2(target[1]-groundPose[1], target[0]-groundPose[0])),2)
		error = -(steering_angle-current_theta)
		if error > 180:
			error = error - 360
		if error < -180:
			error = error + 360
	  	#main controller
	   	Kp=5
		Kd=0.1
		steer = 1*(Kp*error + Kd*(error-lasterror))
	    	dist = round(sqrt(pow((target[0] - groundPose[0]), 2) + pow((target[1] - groundPose[1]), 2)),3)
		obsDist = round(sqrt(pow((obstaclePose[0] - groundPose[0]), 2) + pow((obstaclePose[1] - groundPose[1]), 2)),3)
	    	#to avoid jerks when near to the desired poition: Velocity is controlled 
		velocity = 50 + 200*min(dist, obsDist)  
		if velocity > 255:
			velocity = 255
		#stop at target and obstacle
		if dist < 0.07 or obsDist < 0.3:
			LSpeed = 0
			RSpeed = 0
		#apply steer and high error controller
		else:
			LSpeed = int(-steer+velocity)
			RSpeed = int(+steer+velocity)
			if LSpeed < 0 or error > 60:
				LSpeed = 0
			if RSpeed < 0 or error < -60:
				RSpeed = 0
			if LSpeed > 255:
				LSpeed = 255
			if RSpeed > 255:
				RSpeed = 255
		#drive motors
		MotorRight.setSpeed(RSpeed)
		MotorLeft.setSpeed(LSpeed)
		lasterror=error
		if dist < 0.07:
			if targetIndex == len(targets)-1: #final target
				mode = 3
			elif targetIndex == 2: #stop and rest
				mode = 2
			else:
				lasterror = 0
				targetIndex = targetIndex+1
		if obsDist < 0.3:	#stop and lift
			#targetIndex = 2
			mode = 2
	elif mode == 2: #engage electromagnet
		if flyMode == 0:	#drive
			mode = 1 #drive
		elif flyMode == 1:	#engage
			ElectroMagnet.setSpeed(255)
		elif flyMode == 2:
			ElectroMagnet.setSpeed(0)
		else:	#flyMode == 3	# stay coupled to rest and drive
			ElectroMagnet.setSpeed(255)
			targetIndex = targetIndex+1	#set next target
			mode = 1	#drive
	else: #mode == 3	#end
		time.sleep(1)
		ElectroMagnet.setSpeed(0)

def aero(data): #retrieve UAV pose data
	global aeroPose
	aeroPose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

#def obstacle(data): #retrieve obstacle pose data
#	global obstaclePose
#	obstaclePose=[data.pose.position.x,data.pose.position.y]

def fly(data): #retrieve UAV status	#0 = follow/release
	global flyMode			#1 = lift
	flyMode = data.data		#2 = rest
	rospy.loginfo('Fly Mode = %d' % flyMode)

def turnOffMotors():
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
atexit.register(turnOffMotors)

def listener():
	rospy.init_node('groundNode', anonymous=False) 
	rospy.Subscriber('/groundPose', PoseStamped, ground)
	rospy.Subscriber('/mavros/mocap/pose', PoseStamped, aero)
	#rospy.Subscriber('/obstaclePose', PoseStamped, obstacle)
	rospy.Subscriber('/flyMode', Int8, fly)
	#driver()
	#rospy.Subscriber('/driveMode', Int8, driver)
    
	#gPub = rospy.Publisher('/driveMode', Int8, queue_size = 10)
	#rate = rospy.Rate(100)
	#Mode = Int8()
	#while not rospy.is_shutdown():
	    	#Mode.data = mode
		#gPub.publish(Mode)
		#rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	listener()
	
