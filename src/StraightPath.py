#!/usr/bin/env python

import rospy
import rosbag
from ackermann_msgs.msg import AckermannDriveStamped
import Utils
from std_msgs.msg import String
import matplotlib.pyplot as plt
import numpy as np
import math


PUB_TOPIC = '/car1/mux/ackermann_cmd_mux/input/navigation'
PUB_TOPIC2 = '/msg_test'
PUB_RATE = 3 


# Follows the simulated robot around
class StraightCarLeader:
  '''
  Initializes a CloneFollower object
  In:
    follow_offset: The required x offset between the robot and its clone follower
    force_in_bounds: Whether the clone should toggle between following in front
                     and behind when it goes out of bounds of the map
  '''
  def __init__(self, speed, steering, message):
    self.speed = speed
    self.steering = steering
    self.message = message
    #self.map_img, self.map_info = Utils.get_map (MAP_TOPIC)
    
    # Setup publisher that publishes to PUB_TOPIC
    self.pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=10)
    self.pub2 = rospy.Publisher(PUB_TOPIC2, String, queue_size=10)
    self.count = 0
    self.bufSize = 50 #25
    self.speedArray = []
    self.expSpeedArray = []
    self.speedMsg = 0
    self.timeBuf = []
    self.plotDist = []
    self.plotSize = 2800
    self.Message = 'helloWorld'
    self.repFactor = 3;

    self.DecMsg = ''
    for char in self.message:
        deci = ord(char)
        binary_rep = str(Utils.decimalToBinary(deci))
        count = 8 - len(binary_rep)

        if count > 0:
          rvs_bin = Utils.reverse(binary_rep)
          for i in range(count):
            rvs_bin += "0" 
          binary_rep = Utils.reverse(rvs_bin)

        self.DecMsg = self.DecMsg + binary_rep


    self.set_speed2(self.DecMsg)

    #self.send_message()

  def set_speed(self): 

    rate = rospy.Rate(PUB_RATE)
    switch = 0
    while not rospy.is_shutdown():

      self.timeBuf.append(0)
      
      if len(self.timeBuf) > self.bufSize:

        if switch%2 == 0:
          self.speed = 2.3
        else:
          self.speed = 2.0
        switch += 1
        self.timeBuf = []

      ack_msg = AckermannDriveStamped()

      self.plotDist.append(self.speed)
      if len(self.plotDist) == self.plotSize:
        self.cpyPlot = self.plotDist
        self.plot(self.cpyPlot, self.plotSize)

      ack_msg.header.stamp = rospy.Time.now()
      ack_msg.header.frame_id = '/map'
      ack_msg.drive.steering_angle = steering
      ack_msg.drive.speed = self.speed
      self.pub.publish(ack_msg)

      rate.sleep()

  def set_speed2(self, decodedMsg):
    sent_msg = self.message
    rate = rospy.Rate(PUB_RATE)
    test = String()
    i = 0
    switch = 0
    #while not rospy.is_shutdown():
    for char in decodedMsg:
      #while i > self.bufSize:
      for i in range(self.repFactor):
        if char == '0':
          self.speed = 2.3
        else :
          self.speed = 2.0
      #i = 0
      #i += 1

        test = 'char: '+str(char)+'speed '+str(self.speed)+' message: '+ self.message +' message in bits: ' +self.DecMsg + ' check: '+str(len(self.DecMsg))
        self.pub2.publish(test)

        ack_msg = AckermannDriveStamped()

        self.plotDist.append(self.speed)

        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = '/map'
        ack_msg.drive.steering_angle = steering
        ack_msg.drive.speed = self.speed
        self.pub.publish(ack_msg)

        rate.sleep()

    self.cpyPlot = self.plotDist
    self.plot(self.cpyPlot, self.plotSize)

  def send_message(self):

    rate = rospy.Rate(PUB_RATE)
    act_speed = 2
    ack_msg = AckermannDriveStamped()
    ack_msg.header.stamp = rospy.Time.now()
    ack_msg.header.frame_id = '/map'
    ack_msg.drive.steering_angle = 0



    sent_msg = self.message
    

    test = String()
    test = 'speedArray: ' 
    self.pub2.publish(test)

    for char in sent_msg:
      deci = ord(char)
      binary_rep = str(Utils.decimalToBinary(deci))
      count = 8 - len(binary_rep)

      if count > 0:
        rvs_bin = Utils.reverse(binary_rep)
        for i in range(count):
          rvs_bin += "0" 
        binary_rep = Utils.reverse(rvs_bin)

      switch = 0
      for char in binary_rep:
        if switch % 2 == 0:
          if char == '0':
            act_speed = speed + 0.3
          elif char == '1':
            act_speed = speed + 0.4
        else:
          if char == '0':
            act_speed = speed - 0.3
          elif char == '1':
            act_speed = speed - 0.4
        switch += 1


        self.speedArray.append(act_speed)

    # for eachSpeed in self.speedArray:
    #   for i in range(self.bufSize):
    #     self.expSpeedArray.append(eachSpeed)

    

    # ack_msg.drive.speed = self.expSpeedArray[self.count] 
    # self.count += 1
    # self.pub.publish(ack_msg)
    # rate.sleep()
  def plot(self, y, SIZE):
    t = np.arange(0, len(y), 1)
    plt.plot(t, y, color='b')
    plt.xlabel('fake time')
    plt.ylabel('speed')
    plt.legend()
    plt.show()
    
if __name__ == '__main__':
  speed = 2
  steering = 0 

  rospy.init_node ('StraightCarLeader', anonymous=True) # Initialize the node
  # Populate params with values passed by launch file
  speed = rospy.get_param('speed')
  steering = rospy.get_param('steering')
  message_file = rospy.get_param('message_file')

  with open(message_file) as f:
    message = f.read()   
  scl= StraightCarLeader (speed, steering, message) # Create a car leader
  rospy.spin () # Spin




