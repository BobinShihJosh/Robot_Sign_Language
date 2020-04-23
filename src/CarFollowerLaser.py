#!/usr/bin/env python

import rospy
import Utils
import numpy as np
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
import binascii 
from sensor_msgs.msg import LaserScan

SUB_TOPIC = '/car2/scan'
PUB_TOPIC = '/car2/mux/ackermann_cmd_mux/input/navigation'
PUB_TOPIC2 = '/raw_scans'
MAP_TOPIC = 'static_map'
PUB_RATE = 10

# Follows the simulated robot around
class CarFollower:
  '''
  Initializes a CloneFollower object
  In:
    follow_offset: The required x offset between the robot and its clone follower
    force_in_bounds: Whether the clone should toggle between following in front
                     and behind when it goes out of bounds of the map
  '''
  def __init__(self, follow_offset, force_in_bounds):
    self.follow_offset = follow_offset
    self.force_in_bounds = force_in_bounds
    #self.map_img, self.map_info = Utils.get_map (MAP_TOPIC)
    
    # Setup publisher that publishes to PUB_TOPIC
    self.pub = rospy.Publisher (PUB_TOPIC, AckermannDriveStamped, queue_size=1)
    self.pub2 = rospy.Publisher (PUB_TOPIC2, String, queue_size=10)
    # Setup subscriber that subscribes to SUB_TOPIC and uses the self.update_pose
    # callback
    self.sub = rospy.Subscriber (SUB_TOPIC, LaserScan, self.speed_msg_decode)

    self.range_buf = []
    self.angle_buf = []
    self.still_buf = []

  '''
  In:
    Subscribes to the speed messages of the front car 

  Out:
    Decodes it in to 1 and 0s first and decode the message
  '''
  def speed_msg_decode(self, msg):
    min_range_and_angle = String()
    # find the min between 320-400
    min_range = 1000
    min_ang = 1000
    for angle in range(0,720):
      if msg.ranges[angle] < min_range:
        min_range = msg.ranges[angle]
        min_ang = angle

    self.range_buf.append(min_range)
    self.angle_buf.append(min_ang)

    avg_min_range = 1000
    avg_min_ang = 1000
    if (len(self.range_buf) > 5):
      avg_min_range = str(Utils.Average(self.range_buf))
      avg_min_ang = str(Utils.Average(self.angle_buf))
      self.range_buf = []
      self.angle_bug = []

      min_range_and_angle = 'range: ' + str(avg_min_range) + '  angle: ' + str(avg_min_ang)

    #min_range = str(min(msg.ranges[0:719]))  
    
    #length = str(len(msg.ranges))
      self.pub2.publish(min_range_and_angle)

    # while not rospy.is_shutdown():


        # rate = rospy.Rate(PUB_RATE)
        # act_speed = msg.drive.speed
        # ack_msg = AckermannDriveStamped()
        # ack_msg.header.stamp = rospy.Time.now()
        # ack_msg.header.frame_id = '/map'
        # ack_msg.drive.steering_angle = msg.drive.steering_angle
        # ack_msg.drive.speed = act_speed
        # self.pub.publish(ack_msg)
        # rate.sleep()

    # decode odd to 1, even to 0
    # ascii_byte = String()
    
    # byte = "" 
    # speed = round(10*msg.drive.speed)

    # if speed % 2 == 0:
    #   self.bit_buf.append(0)
    # else:
    #   self.bit_buf.append(1)

    # if len(self.bit_buf) > 7:
    #   for bit in self.bit_buf:
    #     byte += str(self.bit_buf[bit])

    #   self.bit_buf = []
    #   ascii_byte = chr(Utils.binaryToDecimal(int(byte)))
    #   self.pub2.publish(ascii_byte)


      

    
if __name__ == '__main__':
  
  rospy.init_node ('clone_follower', anonymous=True) # Initialize the node
  # Populate params with values passed by launch file
  follow_offset = rospy.get_param ('follow_offset')
  force_in_bounds = rospy.get_param ('force_in_bounds')
  
  cf = CarFollower (follow_offset, force_in_bounds) # Create a clone follower
  rospy.spin () # Spin