#!/usr/bin/env python

import rospy
import Utils
import numpy as np
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
import binascii 

SUB_TOPIC = '/car1/car_pose'
SUB_TOPIC2 = '/car1/mux/ackermann_cmd_mux/input/navigation'
PUB_TOPIC = '/clone_follower_pose/pose'
PUB_TOPIC2 = '/decoded_message'
MAP_TOPIC = 'static_map'


# Follows the simulated robot around
class CloneFollower:
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
    self.map_img, self.map_info = Utils.get_map (MAP_TOPIC)
    
    # Setup publisher that publishes to PUB_TOPIC
    self.pub = rospy.Publisher (PUB_TOPIC, PoseStamped, queue_size=10)
    self.pub2 = rospy.Publisher (PUB_TOPIC2, String, queue_size=10)
    # Setup subscriber that subscribes to SUB_TOPIC and uses the self.update_pose
    # callback
    self.sub = rospy.Subscriber (SUB_TOPIC, PoseStamped, self.update_pose)
    self.sub2 = rospy.Subscriber (SUB_TOPIC2, AckermannDriveStamped, self.speed_msg_decode)

    self.bit_buf = []
  '''
  Given the translation and rotation between the robot and map, computes the pose
  of the clone
  In:
    trans: The translation between the robot and map
    rot: The rotation between the robot and map
  Out:
    The pose of the clone
  '''
  def compute_follow_pose(self, trans, rot):
    theta = Utils.quaternion_to_angle (rot)
    x = self.follow_offset * np.cos (theta) + trans.x
    y = self.follow_offset * np.sin (theta) + trans.y

    return [x, y, theta]

  '''
  In:
    Subscribes to the speed messages of the front car 

  Out:
    Decodes it in to 1 and 0s first and decode the message
  '''
  def speed_msg_decode(self, msg):
    # decode odd to 1, even to 0
    ascii_byte = String()
    
    byte = "" 
    speed = round(10*msg.drive.speed)

    if speed % 2 == 0:
      self.bit_buf.append(0)
    else:
      self.bit_buf.append(1)

    if len(self.bit_buf) > 7:
      for bit in self.bit_buf:
        byte += str(self.bit_buf[bit])

      self.bit_buf = []
      

      ascii_byte = chr(Utils.binaryToDecimal(int(byte)))

      
      #ascii_byte = chr(dec)

      self.pub2.publish(ascii_byte)



   



  '''
  Callback that runs each time a sim pose is received. Should publish an updated
  pose of the clone.
  In:
    msg: The pose of the simulated car. Should be a geometry_msgs/PoseStamped
  '''
  def update_pose(self, msg):    
    # Compute the pose of the clone
    follow_pose = self.compute_follow_pose(msg.pose.position, msg.pose.orientation)

    # Check bounds if required
    if self.force_in_bounds:
      location = Utils.world_to_map (follow_pose, self.map_info)

      if self.map_img[location[1]][location[0]] == 0:
        self.follow_offset *= -1
        follow_pose = self.compute_follow_pose(msg.pose.position, msg.pose.orientation)

    # Setup the outgoing PoseStamped message
    out_pose = PoseStamped ()
    out_pose.header.frame_id = "/map"
    out_pose.header.stamp = rospy.Time.now()

    out_pose.pose.position.x = follow_pose[0]
    out_pose.pose.position.y = follow_pose[1]
    out_pose.pose.orientation = Utils.angle_to_quaternion (follow_pose[2])

    # Publish the clone's pose
    self.pub.publish (out_pose)
    
if __name__ == '__main__':
  follow_offset = -2.5 # The offset between the robot and clone
  force_in_bounds = False # Whether or not map bounds should be enforced
  buffer_size = 100
  rospy.init_node ('clone_follower', anonymous=True) # Initialize the node
  # Populate params with values passed by launch file
  follow_offset = rospy.get_param ('follow_offset')
  force_in_bounds = rospy.get_param ('force_in_bounds')
  
  cf = CloneFollower (follow_offset, force_in_bounds) # Create a clone follower
  rospy.spin () # Spin