#!/usr/bin/env python

import rospy
import rosbag
from ackermann_msgs.msg import AckermannDriveStamped
import Utils

PUB_TOPIC = '/car1/mux/ackermann_cmd_mux/input/navigation'
PUB_RATE = 250 #20 Hz

pub = None
timeBuf = []
bufSize = 25
timeBuf2 = [0,0,0,0,0,0,0,0]
def set_speed(speed, steering, message): 

  rate = rospy.Rate(PUB_RATE)
  act_speed = speed
  ack_msg = AckermannDriveStamped()
  ack_msg.header.stamp = rospy.Time.now()
  ack_msg.header.frame_id = '/map'
  ack_msg.drive.steering_angle = steering



  sent_msg = message
  
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
      timeBuf.append(0)
      if len(timeBuf) >= bufSize:
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
        timeBuf = []
        
        ack_msg.drive.speed = act_speed
        pub.publish(ack_msg)
        rate.sleep()


  """
  # this code makes the car go in a circle at constant velocity(oscilatting)
  # sends 0101010101... 
  switch = 0
  while not rospy.is_shutdown():
    act_speed = speed
    if switch%2 == 0:
      act_speed = speed + 0.1
    else:
      act_speed = speed 
    switch += 1

    ack_msg = AckermannDriveStamped()
    ack_msg.header.stamp = rospy.Time.now()
    ack_msg.header.frame_id = '/map'
    ack_msg.drive.steering_angle = steering
    ack_msg.drive.speed = act_speed
    pub.publish(ack_msg)

    rate.sleep()
  """

if __name__ == '__main__':
  speed = 1
  steering = 0.1

  rospy.init_node('bag_follower', anonymous=True)
  speed = rospy.get_param('speed')
  steering = rospy.get_param('steering')

  pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=10)
  
  message_file = rospy.get_param('message_file')

  with open(message_file) as f:
    message = f.read()

  rospy.sleep(1.0)
  set_speed(speed, steering, message)