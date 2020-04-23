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

PIDdt = float(PIDDistBufSize)/PUB_RATE
dt = distBufSize/PUB_RATE

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
    self.pub = rospy.Publisher (PUB_TOPIC, AckermannDriveStamped, queue_size=10)
    self.pub2 = rospy.Publisher (PUB_TOPIC2, String, queue_size=10)
    self.pub3 = rospy.Publisher (PUB_TOPIC3, String, queue_size=10)
    # Setup subscriber that subscribes to SUB_TOPIC and uses the self.update_pose
    # callback
    self.c1poseSub = message_filters.Subscriber(SUB_TOPIC, PoseStamped)
    self.c2poseSub = message_filters.Subscriber(SUB_TOPIC2, PoseStamped)
    #self.c2Navi = message_filters.Subscriber(SUB_TOPIC3, AckermannDriveStamped)
    self.ts = message_filters.ApproximateTimeSynchronizer([self.c1poseSub, self.c2poseSub], 2, 0.1) 
    self.ts.registerCallback(self.proportional_control)

    self.speed = 0
    self.distBuf = []
    self.syncBuf = []
    self.prevSpd = 0

    self.plotDist = []
    self.cpyPlot = []
    #self.sub = rospy.Subscriber (SUB_TOPIC, PoseStamped, self.cruise_control_msg_decode)
    #self.sub2 = rospy.Subscriber (SUB_TOPIC2, PoseStamped, self.cruise_control_msg_decode)
    # pid BUFFER
    self.PIDBuf = []
    self.dedt = 0
    self.integral = 0

    self.countdT = 1

    self.sigma = 0.000
    self.SNR = 0
    self.evaluateArray = []
    self.cpyPlot2 = []
    self.error = 0
    self.bitError = 0
    self.count2 = 0
    self.SNRBuffer = []
    self.K = 0.1
    self.actualSpeed = 0
    self.actualSpeedBuf = []
  """ 
  1. Get car1 pose and car2 pose, compute their distance continuously
  2. Add Noise to that distance data
  3. Create a servo-ing loop that:
    1. Compares computed distance with offset distance.
    2. If compdute distance > offset distance : increase speed by a certain amount 
    3. If computed distance < offset distance : decrease speed by a certain amount
    4. Goal is to try and keep computed distance == offset distance. 
  4. if car1 alters it's speed, the distance between car1 and car2 will change and car2 will alter its speed.
  5. if the altered speed of car2 is the same as car 1, message succesfully sent.
  """


  def proportional_control(self, car1, car2):
    test = String()
    ack_msg = AckermannDriveStamped()
    rate = rospy.Rate(PUB_RATE)

    raw_distance = self.get_distance(car1.pose.position.x, car1.pose.position.y, 
                                            car2.pose.position.x, car2.pose.position.y) 

    #Adding gaussian noise to signal 
    distance = np.random.normal(raw_distance, self.sigma, 1)
    
    error = raw_distance - self.follow_offset
    self.speed = 5*(error)
    tmpSpeed = 0;
    
    # simulate actual speed of car 1########################################################






    ########################################################################################

    if self.speed > 1.6:
      if self.speed >= 2.15:
        tmpSpeed = 2.3
      else: 
        tmpSpeed = 2
    elif self.speed < -1.6:
      if self.speed <= -2.15:
        tmpSpeed = -2.3
      else:
        tmpSpeed = -2

    ack_msg.header.stamp = rospy.Time.now()
    ack_msg.header.frame_id = '/map'
    ack_msg.drive.steering_angle = 0
    ack_msg.drive.speed = self.speed   

    self.pub.publish(ack_msg)

    test = 'Speed: ' + str(self.speed) + ' Distance: ' + str(distance) + ' error: ' + str(error)

    self.pub2.publish(test)

    self.plotDist.append(self.speed)
    if len(self.plotDist) == 150:
      self.cpyPlot = self.plotDist
      self.plot(self.cpyPlot) 


  def cruise_control(self, car1, car2):
    test = String()
    ack_msg = AckermannDriveStamped()
    rate = rospy.Rate(PUB_RATE)

    raw_distance = self.get_distance(car1.pose.position.x, car1.pose.position.y, 
                                            car2.pose.position.x, car2.pose.position.y) 

    #Adding gaussian noise to signal 
    distance = np.random.normal(raw_distance, self.sigma, 1)
    self.SNRBuffer.append((distance/(distance-raw_distance)))
    


    


#================================= PID CONRTOLLER (PD)

    # offSet = distance - 3
    # self.PIDBuf.append(offSet)


    # if len(self.PIDBuf) >= PIDDistBufSize:
    #   self.dedt = (self.PIDBuf[PIDDistBufSize-1] - self.PIDBuf[0])/(PIDdt)

    #   self.integral = 0.01*self.PIDBuf[0] + ((self.PIDBuf[4]-self.PIDBuf[0])*0.01/2)
    #   self.PIDBuf = []
    #   #proportional gain = how strongly the car wants to to desired offset distance (accel)
    #   #derivative gain = how strongly the car resists too fast changes 
       
    #   # for bufsize >=5 dt = 0.01 use kp = 0.1 kd = 0.01 
    # self.speed += 0.05*offSet + 0.0*self.integral + 0.0035*self.dedt

    # # self.plotDist.append(self.speed)
    # # if len(self.plotDist) == 50:
    # #   self.cpyPlot = self.plotDist
    # #   self.plot(self.cpyPlot)

    # ack_msg.header.stamp = rospy.Time.now()
    # ack_msg.header.frame_id = '/map'
    # ack_msg.drive.steering_angle = 0
    # ack_msg.drive.speed = self.speed  

    # self.pub.publish(ack_msg)
    # test = 'car2 speed: ' + str(self.speed) + '   distance : ' + str(distance) + '   offset' + str(offSet)
    # self.pub2.publish(test)
    #=========================================================
    
    #####################################################################################################
    self.distBuf.append(distance)
    bufLength = len(self.distBuf)
    
    
    ##############=====================
    #detect change to sync with change of speed
    if bufLength >= 2:
      diffLog = self.distBuf[self.countdT] - self.distBuf[self.countdT-1]

       

      if abs(diffLog) > 0.0075:
        self.countdT = 0
        deltaD = self.distBuf[len(self.distBuf)-1] - self.distBuf[0]
        deltaT = float(bufLength) / PUB_RATE
        AVGSpeed = (deltaD/deltaT)/5 + self.prevSpd

        if AVGSpeed > 0.6:
          if AVGSpeed >= 2.15:
            AVGSpeed = 2.3
          else: 
            AVGSpeed = 2

        self.distBuf = []
        self.speed = AVGSpeed
        self.prevSpd = AVGSpeed

      self.countdT += 1 

      test = 'Car2 Speed: ' + str(self.speed) + ' distance : ' + str(distance)
      self.pub2.publish(test)

    # self.plotDist.append(self.speed)
    # if len(self.plotDist) == 100:
    #   self.cpyPlot = self.plotDist
    #   self.plot(self.cpyPlot) 

     

    ack_msg.header.stamp = rospy.Time.now()
    ack_msg.header.frame_id = '/map'
    ack_msg.drive.steering_angle = 0
    ack_msg.drive.speed = self.speed  

    self.pub.publish(ack_msg)
    
    # -----------------evaluation----------------
    self.evaluateArray.append(self.speed)

    if len(self.evaluateArray) >= 200:
      self.cpyPlot2 = self.evaluateArray
      self.evaluate(self.cpyPlot2) 
    
    #######################################################################################################

  def plot(self, y):
    t = np.arange(0, 150, 1)
    plt.plot(t, y, color='b')
    plt.xlabel('fake time')
    plt.ylabel('speed')
    plt.legend()
    plt.show()

  def evaluate(self, signal):
    perfectSignal = []
    startPoint = 0
    for i in range(len(signal)):
      if signal[i] >= 2.0 and signal[i] <= 2.3:
        startPoint = i
        break


    if signal[startPoint] == 2:
      for i in range(50):
        perfectSignal.append(2.0)
        perfectSignal.append(2.0)
        perfectSignal.append(2.3)
        perfectSignal.append(2.3)
    else:
      for i in range(50):
        perfectSignal.append(2.3)
        perfectSignal.append(2.3)
        perfectSignal.append(2.0)
        perfectSignal.append(2.0)

    t = np.arange(0, 200, 1)

    for i in range(len(signal)-21):
      absErr = abs(signal[20+i] - perfectSignal[20+i])
      if absErr != 0:
        self.error += absErr 
        self.count2 += 1
        if self.count2 >= 2:
          self.count2 = 0
          self.bitError += 1
       

    self.error = self.error/(200*2.15)
    self.bitError = self.bitError
    print(self.error)
    error = String()

    SUM = 0
    for i in range(len(self.SNRBuffer)):
      SUM += (self.SNRBuffer[i])

    self.SNR = np.log(SUM/len(self.SNRBuffer))

    self.pub3.publish('speed error: ' + str(self.error) + '  bit Error:'+str(self.bitError) + ' snr: ' + str(self.SNR))

    plt.plot(t, perfectSignal, label='perfect signal', color='r')
    plt.plot(t, signal, label='observed_signal', color='b')
    plt.xlabel('time')
    plt.ylabel('speed(representing each bit)')
    plt.show()
    



  def get_distance(self, x1, y1, x2, y2):
    dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    return dist
  '''
  In:
    Subscribes to the speed messages of the front car 

  Out:
    Decodes it in to 1 and 0s first and decode the message
  '''
  # def cruise_control_msg_decode(self, msg):
  #   min_range_and_angle = String()
  #   # find the min between 320-400
  #   min_range = 1000
  #   min_ang = 1000
  #   for angle in range(0,720):
  #     if msg.ranges[angle] < min_range:
  #       min_range = msg.ranges[angle]
  #       min_ang = angle

  #   self.range_buf.append(min_range)
  #   self.angle_buf.append(min_ang)

  #   avg_min_range = 1000
  #   avg_min_ang = 1000
  #   if (len(self.range_buf) > 5):
  #     avg_min_range = str(Utils.Average(self.range_buf))
  #     avg_min_ang = str(Utils.Average(self.angle_buf))
  #     self.range_buf = []
  #     self.angle_bug = []

  #     min_range_and_angle = 'range: ' + str(avg_min_range) + '  angle: ' + str(avg_min_ang)

  #   #min_range = str(min(msg.ranges[0:719]))  
    
  #   #length = str(len(msg.ranges))
  #     self.pub2.publish(min_range_and_angle)

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

    


      

    
if __name__ == '__main__':
  follow_offset = 3 # The offset between the robot and clone
  force_in_bounds = False # Whether or not map bounds should be enforced

  rospy.init_node ('Car_follower', anonymous=True) # Initialize the node
  # Populate params with values passed by launch file
  follow_offset = rospy.get_param ('follow_offset')
  force_in_bounds = rospy.get_param ('force_in_bounds')
  
  cf = CarFollower (follow_offset, force_in_bounds) # Create a clone follower
  rospy.spin () # Spin
