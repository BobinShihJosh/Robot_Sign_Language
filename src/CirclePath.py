#!/usr/bin/env python

import rospy
import rosbag
from ackermann_msgs.msg import AckermannDriveStamped

PUB_TOPIC = '/car1/mux/ackermann_cmd_mux/input/navigation'
PUB_RATE = 100

pub = None

def set_speed(speed, steering): 

  rate = rospy.Rate(PUB_RATE)
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

if __name__ == '__main__':
  speed = 1
  steering = 0.1

  rospy.init_node('bag_follower', anonymous=True)
  speed = rospy.get_param('speed')
  steering = rospy.get_param('steering')

  pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=10)
  
  set_speed(speed, steering)