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

SUB_TOPIC = '/car2/mux/ackermann_cmd_mux/input/navigation'
PUB_TOPIC = '/Decoded_Message'

bit_buf = []

class decode_message:
	def __init__(self):
		self.pub = rospy.Publisher(PUB_TOPIC, String, queue_size=10)
		self.sub = rospy.Subscriber(SUB_TOPIC, AckermannDriveStamped, self.decode)
	  	self.bit_buf = []

	def decode(self, msg): 
		

		#decode odd to 1, even to 0
	    ascii_byte = String()
	    
	    byte = "" 
	     # speed = round(10*msg.drive.speed)
	    avgSpeed = msg.drive.speed
	    if avgSpeed > 0.6:
          	if avgSpeed >= 1.05: avgSpeed = 1.1
          	else: avgSpeed = 1.0

	    self.pub.publish(str(avgSpeed))



	    # if speed % 2 == 0:
	    #   self.bit_buf.append(0)
	    # else:
	    #   self.bit_buf.append(1)

	    # if len(self.bit_buf) > 7:
	    #   for bit in self.bit_buf:
	    #     byte += str(self.bit_buf[bit])

	    #   self.bit_buf = []
	    #   ascii_byte = chr(Utils.binaryToDecimal(int(byte)))
	    #   self.pub.publish(ascii_byte)

  

  


if __name__ == '__main__':

  rospy.init_node('Message_decoder', anonymous=True)
  
  
  dm = decode_message()
  rospy.spin()