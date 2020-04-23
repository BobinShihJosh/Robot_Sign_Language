#!/usr/bin/env python

import rospy
import Utils
import numpy as np
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
import binascii 
from sensor_msgs.msg import LaserScan
import message_filters
import math
import matplotlib.pyplot as plt
import numpy as np

SUB_TOPIC = '/car1/car_pose'
SUB_TOPIC2 = '/car2/car_pose'
SUB_TOPIC3 = '/car2/mux/ackermann_cmd_mux/input/navigation'
PUB_TOPIC = '/car2/mux/ackermann_cmd_mux/input/navigation'
PUB_TOPIC2 = '/usedata_to_view'
PUB_TOPIC3 = '/error'
MAP_TOPIC = 'static_map'
PUB_RATE = 250.0
distBufSize = 3.0
PIDDistBufSize = 10

class CarFollower2:
	def __init__(self):

		self.pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size = 10)
		self.pub2 = rospy.Publisher(PUB_TOPIC2, String, queue_size = 10)
		# self.sub = rospy.Subscriber(SUB_TOPIC, PoseStamped, self.pass1)
		# self.sub2 = rospy.Subscriber(SUB_TOPIC2, PoseStamped, self.pass2)

		self.c1poseSub = message_filters.Subscriber(SUB_TOPIC, PoseStamped)
	  	self.c2poseSub = message_filters.Subscriber(SUB_TOPIC2, PoseStamped)
	    #self.c2Navi = message_filters.Subscriber(SUB_TOPIC3, AckermannDriveStamped)
	  	self.ts = message_filters.ApproximateTimeSynchronizer([self.c1poseSub, self.c2poseSub], 2, 0.03) 
	  	self.ts.registerCallback(self.proportional_control)
	  	self.follow_offset = 3.0
		self.prevPoseX = 0
		self.prevPoseY = 0
		self.currPoseX = 0
		self.currPoseY = 0

		self.currTime = 0;
		self.prevTime = 0;

		self.car1Queue = [0,0]
		self.car2Queue = [0,0]

		self.syncSpeed = []
		self.speed = 0
		self.sigma = 0.0001
		self.outMsg = []

		now = rospy.Time.now()
		car1PoseArriveTime = now.to_nsec()
		self.plotDist = []

		self.car1SyncTime = [car1PoseArriveTime%(10**10)]
		self.car2SyncTime = [car1PoseArriveTime%(10**10)]
		self.car1Lead = False

	def proportional_control(self, car1, car2):
		test = String()
		ack_msg = AckermannDriveStamped()
		now = rospy.Time.now()
		car1PoseArriveTime = now.to_nsec()

		# Getting the timestamp in secs + nano secs 
		car1ts = ((car1.header.stamp.secs%10)*10**9) + car1.header.stamp.nsecs
		car2ts = ((car2.header.stamp.secs%10)*10**9) + car2.header.stamp.nsecs

		# Update append the car1 and car2 buffer
		self.car1Queue.append(car1.pose.position.x)
		self.car1Queue.append(car1.pose.position.y)

		self.car2Queue.append(car2.pose.position.x)
		self.car2Queue.append(car2.pose.position.y)

		self.car1SyncTime.append(car2ts)
		self.car2SyncTime.append(car1ts)

		# calculate the speed of the car with earlier time stamp and position so we can 
		# estimate where it is going to be at the more current timestamp
		if car1ts >= car2ts:
			self.car1Lead = False
			dT = (car1ts - car2ts)#/(1.0*10**9)

			dsT = self.car1SyncTime[len(self.car1SyncTime)-1] - self.car1SyncTime[len(self.car1SyncTime)-2]
			#dsT = tdsT/(1.0*10**9)
			self.syncSpeed.append((self.get_distance(self.car2Queue[0], self.car2Queue[1], self.car2Queue[2], self.car2Queue[3]))/float(dsT))

		else:
			self.car1Lead = True 
			dT = (car2ts - car1ts)#/(1.0*10**9)

			dsT = self.car2SyncTime[len(self.car2SyncTime)-1] - self.car2SyncTime[len(self.car2SyncTime)-2]
			#dsT = tdsT/(1.0*10**9)
			self.syncSpeed.append((self.get_distance(self.car1Queue[0], self.car1Queue[1], self.car1Queue[2], self.car1Queue[3]))/float(dsT))


		# this is the addition distance that is missed from the approximate time bug 
		deltaD = self.syncSpeed[len(self.syncSpeed)-1] * dT 
		 
		## now we test everything else with the actual speed modification
		rawDistance = self.get_distance(car1.pose.position.x, car1.pose.position.y, 
                                            car2.pose.position.x, car2.pose.position.y)
		

		if self.car1Lead:
			self.syncedDistance = rawDistance + deltaD
		else:
			self.syncedDistance = rawDistance - deltaD

		# adding gaussian noise 	
		noisyDistance = np.random.normal(self.syncedDistance, self.sigma, 1)	

		test = 'rD '+str(rawDistance)+' sD '+str(self.syncedDistance)+' dT ' +str(dT)+' dD '+str(deltaD)+' car1: '+str(car1ts)+' car2: '+str(car2ts)+' syncspeed '+str(self.syncSpeed)+' dsT '+str(dsT)+ ' car1T '+str(self.car1SyncTime)+' car2T '+str(self.car2SyncTime)

		error = noisyDistance - self.follow_offset
		self.speed = 5.0*error

		ack_msg.header.stamp = rospy.Time.now()
		ack_msg.header.frame_id = '/map'
		ack_msg.drive.steering_angle = 0
		ack_msg.drive.speed = self.speed

		self.pub.publish(ack_msg)

		#decodeSpeed(self, self.speed)

		# Pop out old values FIFO 
		self.car1Queue.pop(0)
		self.car1Queue.pop(0)

		self.car2Queue.pop(0)
		self.car2Queue.pop(0)

		self.car1SyncTime.pop(0)
		self.car2SyncTime.pop(0)
		self.syncSpeed.pop(0)

		self.plotDist.append(self.speed)
		if len(self.plotDist) == 480:
			self.cpyPlot = self.plotDist
			self.plot(self.cpyPlot) 

		test += 'out message' + str(self.outMsg) + ' length: '+str(len(self.outMsg))

		self.pub2.publish(test)

	# def decodeSpeed(self, currSpeed):
	# 	if currSpeed > 2.15:


	# def pass1(self, msg):
	# 	# now = rospy.Time.now()
	# 	# car1PoseArriveTime = now.to_nsec()

	# 	self.sync_message(((msg.header.stamp.secs%10)*10**9) + msg.header.stamp.nsecs, msg.pose.position.x, msg.pose.position.y,0, 0, 0)

	# def pass2(self, msg):
	# 	# now = rospy.Time.now()
	# 	# car2PoseArriveTime = now.to_nsec()
		
	# 	self.sync_message(0, 0, 0, ((msg.header.stamp.secs%10)*10**9) + msg.header.stamp.nsecs, msg.pose.position.x, msg.pose.position.y)


	# def sync_message(self, car1Time, car1X, car1Y, car2Time, car2X, car2Y):
	# 	test = String()

	# 	if car1Time != 0 and car2Time != 0:
	# 		test = 'simult'

	# 	timeDiff = 0
	# 	car1 = ''
	# 	car2 = ''

	# 	if car1Time != 0:
	# 		car1 = ' car 1: ' + str(car1Time)
	# 		self.car1Queue.append(car1Time)

	# 	if car2Time != 0:
	# 		car2 = '  car 2: ' + str(car2Time)
	# 		self.car2Queue.append(car2Time)

		

	# 	test = 'time diff 2 : ' + str(abs(self.car2Queue[0] - self.car1Queue[0]))
		
	# 	# if car1Time != 0:
	# 	# 	self.car1Queue.append(car1Time)
	# 	# 	self.car1Queue.append(car1X)
	# 	# 	self.car1Queue.append(car1Y)
	# 	# 	self.tempTime1 = car1Time
	# 	# if car2Time != 0:	
	# 	# 	self.car2Queue.append(car2Time)
	# 	# 	self.car2Queue.append(car2X)
	# 	# 	self.car2Queue.append(car2Y)
	# 	# 	self.tempTime2 = car2Time
		
	# 	# self.timeDiff = 0

	# 	# if len(self.car1Queue) > 0 and len(self.car2Queue) > 0:
	# 	# 	self.timeDiff = self.car1Queue[0] - self.car2Queue[3]

	# 	# if len(self.car1Queue) > 3 and len(self.car2Queue) > 3:
	# 	# 	self.car1Queue = []
	# 	# 	self.car2Queue = []

	# 	self.pub2.publish(test)

	# 	# empty queues 
	# 	if len(self.car1Queue) >= 2:
	# 		self.car1Queue = []
	# 	if len(self.car2Queue) >= 2:
	# 		self.car2Queue = []

	# def speed_cruise(self, msg):

	# 	# Update current position and times 
	# 	self.currPoseX = msg.pose.position.x
	# 	self.currPoseY = msg.pose.position.y
	# 	now = rospy.Time.now()
	# 	self.currTime = now.to_nsec()

	# 	# Compute distance between time interval 
	# 	rawDistance = self.get_distance(self.currPoseX, self.currPoseY, 
	# 									self.prevPoseX, self.prevPoseY)
	# 	# Compute length of time interval 
	# 	duration = self.currTime - self.prevTime
	# 	# Compute speed of the car based on delta distance over that time interval
	# 	speed = self.get_speed(rawDistance, duration)
	# 	# if speed >= 2.0:
	# 	# 	speed = 2.1
	# 	# else:
	# 	# 	speed = 1.9
	

	# 	# Update previous position and times
	# 	self.prevPoseX = self.currPoseX
	# 	self.prevPoseY = self.currPoseY
	# 	self.prevTime = self.currTime

	# 	#### testing by publishing to topic
	# 	test = String()
	# 	test = 'Speed: ' + str(speed) + ' duration : ' + str(duration) + ' rawDistance: ' + str(rawDistance)
	# 	self.pub2.publish(test)
	def plot(self, y):
		
		for i in xrange(0, len(y), 3):
			if y[i] >= 2.15:
				self.outMsg.append(1) 
			else: 
				self.outMsg.append(0)
			

		t = np.arange(0, len(y), 1)
		plt.plot(t, y, color='b')
		plt.xlabel('number of distance measurements')
		plt.ylabel('speed')
		plt.legend()
		plt.show()

	def get_speed(self, distance, time):
		return 10**9*(distance/time)

	def get_distance(self, x1, y1, x2, y2):
	    dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
	    return dist


if __name__ == '__main__':

  follow_offset = 3 # The offset between the robot and clone
  force_in_bounds = False # Whether or not map bounds should be enforced

  rospy.init_node ('CarFollower2', anonymous=True) # Initialize the node
  # Populate params with values passed by launch file
  follow_offset = rospy.get_param ('follow_offset')
  force_in_bounds = rospy.get_param ('force_in_bounds')
  
  cf = CarFollower2() # Create a clone follower
  rospy.spin () # Spin