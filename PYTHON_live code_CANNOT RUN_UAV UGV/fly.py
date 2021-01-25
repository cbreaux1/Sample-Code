#!/usr/bin/env python
import rospy
import thread
import threading
import time
import mavros
from math import *
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
gPose = [2.0,-11.0,0.10]
#gPose = [0.0, -5.3, 0.1] #take out
oPose = [0.0,-5.0,0.0]
global maxerror 
maxerror = 0.1
aPub = rospy.Publisher('/flyMode', Int8, queue_size = 10)

class SetpointPosition:
	def __init__(self):
	        self.x = 0.0
	        self.y = 0.0
	        self.z = 0.0
	        # publisher for mavros/setpoint_position/local
	        self.pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10) 
	        # subscriber for mavros/local_position/local
	        self.sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.reached)
	 
		try:
			thread.start_new_thread(self.navigate, ())
	        except:
			fault('Error: Unable to start thread')
        	self.done = False
        	self.done_evt = threading.Event()
 
	def navigate(self):
	        rate = rospy.Rate(100)   
	        msg = SP.PoseStamped(header=SP.Header(frame_id='base_footprint', stamp=rospy.Time.now()), )
	
	        while not rospy.is_shutdown():
	        	msg.pose.position.x = self.x
	        	msg.pose.position.y = self.y
            		msg.pose.position.z = self.z
 
			#lock yaw/heading to north.
			yaw_degrees = 0  
			yaw = radians(yaw_degrees)
			quaternion = quaternion_from_euler(0, 0, yaw)
			msg.pose.orientation = SP.Quaternion(*quaternion)
 
			self.pub.publish(msg)
			rate.sleep()
 
	def set(self, x, y, z, delay=0, wait=True):
	        self.done = False
	        self.x = x
	        self.y = y
	        self.z = z
 
	        if wait:
	        	rate = rospy.Rate(100)
            		while not self.done and not rospy.is_shutdown():
                		rate.sleep()
 
        	time.sleep(delay)
 
	def reached(self, topic):
		def is_near(msg, x, y):
			rospy.logdebug('Position %s: local: %d, target: %d, abs diff: %d', msg, x, y, abs(x - y))
			return abs(x - y) < maxerror 
		if is_near('X', topic.pose.position.x, self.x) and is_near('Y', topic.pose.position.y, self.y) and is_near('Z', topic.pose.position.z, self.z):
       			self.done = True
            		self.done_evt.set()

def ground(data):
	global gPose
	gPose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

def obstacle(data):
	global oPose
	oPose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

def sequence():
	global maxerror
	rospy.init_node('aeroNode')
	rospy.Subscriber('/groundPose', PoseStamped, ground)
	rospy.Subscriber('/obstaclePose', PoseStamped, obstacle)
	aPub.publish(0) #follow
	global setpoint
	setpoint = SetpointPosition()
	#maxerror = 0.2 #take out
	#setpoint.set(gPose[0], gPose[1], gPose[2]+0.1, 5) #take out
	#aPub.publish(1) #take out
	#setpoint.set(gPose[0], gPose[1], gPose[2]+0.1, 3) #take out
	rospy.loginfo('Takeoff')
	##setpoint.set(1, -11, 1.5, 2)
	##rospy.loginfo('Follow')
	##while sqrt(pow((gPose[0]-oPose[0]), 2) + pow((gPose[1]-oPose[1]), 2)) > 0.3:
	##	setpoint.set(gPose[0], gPose[1], gPose[2]+1.5, 0)
	##rospy.loginfo('Descend')
	##aPub.publish(1) #engage
	##cPose = [gPose[0], gPose[1], gPose[2]] 
	cPose = [0, -5.3, 0.11]  #take out
	##setpoint.set(cPose[0], cPose[1], cPose[2]+1, 0)
	##setpoint.set(cPose[0], cPose[1], cPose[2]+0.5, 0)
	##maxerror = 0.05
	##setpoint.set(cPose[0], cPose[1], cPose[2]+0.2, 0)
	##maxerror = 0.01
	##setpoint.set(cPose[0], cPose[1], cPose[2]+0.12, 0)
	##maxerror = 0.13
	##setpoint.set(cPose[0], cPose[1], cPose[2], 2)
	rospy.loginfo('Lift')
	##maxerror = 0.2
	##setpoint.set(cPose[0], cPose[1], cPose[2]+0.6, 0)
	##setpoint.set(0, -4.5, 0.5, 0)
	rospy.loginfo('Lower')
	#maxerror = 0.2
	##setpoint.set(0, -4.5, 0.2, 0.25)
	##setpoint.set(0, -4.5, 0.1, 0)
	##setpoint.set(0, -4.5, 0, 1)
	aPub.publish(2) #release
	##setpoint.set(0, -4.5, 0, 1)
	maxerror = 0.1
	rospy.loginfo('Takeoff')
	setpoint.set(0, -4.5, 1.5, 0)
	aPub.publish(0) #follow
	rospy.loginfo('Follow')
	while sqrt(pow((gPose[0]-(0)), 2) + pow((gPose[1]-(-3)), 2)) > 0.07:
		setpoint.set(gPose[0], gPose[1], gPose[2]+1.5, 0)
		aPub.publish(0) #takeout
	rospy.loginfo('Descend')
	aPub.publish(1) #engage
	cPose = [gPose[0], gPose[1], gPose[2]] 
	setpoint.set(cPose[0], cPose[1], cPose[2]+1, 0)
	setpoint.set(cPose[0], cPose[1], cPose[2]+0.5, 0)
	maxerror = 0.05
	setpoint.set(cPose[0], cPose[1], cPose[2]+0.2, 0)
	maxerror = 0.01
	setpoint.set(cPose[0], cPose[1], cPose[2]+0.12, 0)
	maxerror = 0.2
	setpoint.set(cPose[0], cPose[1], cPose[2], 2)
	rospy.loginfo('Rest')
	aPub.publish(3) #rest
	self.set_mode("MANUAL", 5)
	self.set_arm(False, 5)

if __name__ == '__main__':
    try:
	sequence()
    except rospy.ROSInterruptException:
	pass
