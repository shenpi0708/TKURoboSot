#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import time
import numpy as np
import time
import message_filters
import actionlib
import strategy.msg
from simple_pid import PID
from imu_3d.msg import inertia
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from vision.msg import Object
from vision.msg import Two_point
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
from strategy.msg import RobotState
from std_srvs.srv import *

## Rotate 90 for 6th robot
## DO NOT CHANGE THIS VALUE
ROTATE_V_ANG = 90

## Strategy Inputs
VISION_TOPIC = "vision/object"
POSITION_TOPIC = "akf_pose"
IMU            = "imu_3d"
RE            = "requestsignal"
PASSREQUESTPass = "PassRequestPass"
PASSREQUESTCatch = "PassRequestCatch"
## Strategy Outputs
STRATEGY_STATE_TOPIC = "strategy/state"
CMDVEL_TOPIC = "motion/cmd_vel"
SHOOT_TOPIC  = "motion/shoot"
REQUEST_SIGNAL = "requestsignal"
PASS_REQUESTPass = "PassRequestPass"
PASS_REQUESTCatch = "PassRequestCatch"
class Robot(object):
  
  __twopoint_info = {'Blue':{'right' : 0,'left' : 0},
                     'Yellow':{'right' : 0,'left' : 0}}

  ball_last_time = time.time()
  sync_last_time = time.time()
  passing_last_time = time.time()

  __robot_info  = {'location' : {'x' : 0, 'y' : 0, 'yaw' : 0},
                   'imu_3d' : {'yaw' : 0}}
  __object_info = {'ball':{'dis' : 0, 'ang' : 0, 'global_x' : 0, 'global_y' : 0, \
                           'speed_x': 0, 'speed_y': 0, 'speed_pwm_x': 0, 'speed_pwm_y': 0},
                   'Blue':{'dis' : 0, 'ang' : 0},
                   'Yellow':{'dis' : 0, 'ang' : 0},
                   'velocity' : 0 }
  __obstacle_info = {'angle' : {'min' : 0, 'max' : 0, 'increment' : math.pi / 180 * 3},
		                 'scan' : 0,
		                 'range' : {'min' : 0, 'max' : 0},
                     'ranges' : [0],
		                 'intensities' : [0]}

  __ball_is_handled = False

  robot1 = {'state': '', 'ball_is_handled': False, 'ball_dis': 0, 'position': {'x': 0, 'y': 0, 'yaw': 0}}
  robot2 = {'state': '', 'ball_is_handled': False, 'ball_dis': 0, 'position': {'x': 0, 'y': 0, 'yaw': 0}}
  robot3 = {'state': '', 'ball_is_handled': False, 'ball_dis': 0, 'position': {'x': 0, 'y': 0, 'yaw': 0}}
  near_robot = {'state': '', 'ball_is_handled': False, 'ball_dis': 0, 'position': {'x': 0, 'y': 0, 'yaw': 0}}
  near_robot_ns = ""
  r1_role = ""
  r2_role = ""
  r3_role = ""
  ## Configs
  __minimum_w = 0
  __maximum_w = 0
  __minimum_v = 0
  __maximum_v = 0
  __handle_dis = 0
  __handle_ang = 0
  Kp_v = 1.5
  Ki_v = 0.0
  Kd_v = 0.1
  Cp_v = 0
  Kp_w = 0.5
  Ki_w = 0.0
  Kd_w = 0.1
  Cp_w = 0
  __requestsignalin =True
  requestsignal =  True

  pid_v = PID(Kp_v, Ki_v, Kd_v, setpoint=Cp_v)
  pid_v.output_limits = (-1*__maximum_v, __maximum_v)
  pid_v.auto_mode = True
  pid_w = PID(Kp_w, Ki_w, Kd_w, setpoint=Cp_w)
  pid_w.output_limits = (-1*__maximum_w, __maximum_w)
  pid_w.auto_mode = True

  is_catch = False
  is_pass = False

  PassRequestPass = False
  R1_PassRequestPass = False
  R2_PassRequestPass = False
  R3_PassRequestPass = False
  Other_PassRequestPass =False
  PassRequestCatch = False
  R2_PassRequestCatch = False
  R3_PassRequestCatch = False
  Other_PassRequestCatch = False
  canpassball=True
  ball_passingpass = False
  ball_passingcatch = False
  def TuningVelocityContorller(self, p, i, d, cp = Cp_v):
    self.pid_v.setpoint = cp
    self.pid_v.tunings = (p, i, d)

  def TuningAngularVelocityContorller(self, p, i, d, cp = Cp_w):
    self.pid_w.setpoint = cp
    self.pid_w.tunings = (p, i, d)

  def ChangeVelocityRange(self, m, M):
    self.__minimum_v = m
    self.pid_v.output_limits = (-1*M, M)

  def ChangeAngularVelocityRange(self, m, M):
    self.__minimum_w = m
    self.pid_w.output_limits = (-1*M, M)

  def ChangeBallhandleCondition(self, dis, ang):
    self.__handle_dis = dis
    self.__handle_ang = ang

  def ShowRobotInfo(self):
    print("Robot informations: {}".format(self.__robot_info))
    print("Objects informations: {}".format(self.__object_info))
    print("Obstacles informations: {}".format(self.__obstacle_info))
    print(self.robot2)
    print(self.robot3)

  def __init__(self, sim = False):

    rospy.Subscriber(VISION_TOPIC, Object, self._GetVision)
    rospy.Subscriber(POSITION_TOPIC, PoseWithCovarianceStamped, self._GetPosition)
    rospy.Subscriber('BlackRealDis', Int32MultiArray, self._GetBlackItemInfo)
    rospy.Subscriber("requestsignal", Bool, self._Getre)
    self.MotionCtrl = self.RobotCtrlS
    self.RobotShoot = self.RealShoot
    self.cmdvel_pub = self._Publisher(CMDVEL_TOPIC, Twist)
    self.state_pub  = self._Publisher(STRATEGY_STATE_TOPIC, RobotState)
    self.shoot_pub  = self._Publisher(SHOOT_TOPIC, Int32)
    self.requestsignal_pub = self._Publisher(REQUEST_SIGNAL,Bool)
    self.PassRequestPass_pub = self._Publisher(PASS_REQUESTPass,Bool)
    self.PassRequestCatch_pub = self._Publisher(PASS_REQUESTCatch,Bool)
    robot2_sub = message_filters.Subscriber('/robot2/strategy/state', RobotState)
    robot3_sub = message_filters.Subscriber('/robot3/strategy/state', RobotState)
    ts = message_filters.ApproximateTimeSynchronizer([robot2_sub, robot3_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(self.MulticastReceiver)
    robot1_subPass = message_filters.Subscriber('/robot1/PassRequestPass', Bool)
    robot2_subPass = message_filters.Subscriber('/robot2/PassRequestPass', Bool)
    robot3_subPass = message_filters.Subscriber('/robot3/PassRequestPass', Bool)
    tsPass = message_filters.ApproximateTimeSynchronizer([robot1_subPass,robot2_subPass, robot3_subPass], 10, 0.1, allow_headerless=True)
    tsPass.registerCallback(self.MulticastReceiverPass)
    robot2_subCatch = message_filters.Subscriber('/robot2/PassRequestCatch', Bool)
    robot3_subCatch = message_filters.Subscriber('/robot3/PassRequestCatch', Bool)
    tsCatch = message_filters.ApproximateTimeSynchronizer([robot2_subCatch, robot3_subCatch], 10, 0.1, allow_headerless=True)
    tsCatch.registerCallback(self.MulticastReceiverCatch)
    s = rospy.Service('passing_action', Trigger, self._PassingServer)
    
    if not sim :
      rospy.Subscriber('interface/Two_point', Two_point, self._GetTwopoint)
      rospy.Subscriber(IMU, inertia, self._GetImu)
      self.RobotBallHandle = self.RealBallHandle
    else:
      self.sim_hold_pub = rospy.Publisher('motion/hold', Bool, queue_size=1)
      self.RobotBallHandle = self.SimBallHandle
      rospy.Subscriber("BallIsHandle", Bool, self._CheckBallHandle)
      self.TuningVelocityContorller(1, 0, 0)
      self.TuningAngularVelocityContorller(0.1, 0, 0)

  def _PassingServer(self, req):
    self.MyRole = "Catcher"
    resp = TriggerResponse()
    resp.success = True
    resp.message = "Got it"
    return resp

  def PassingTo(self, catcher_ns):
    self.MyRole = "Passer"
    if catcher_ns is "nearest":
      server = self.near_robot_ns + 'passing_action'
    else:
      server = catcher_ns + 'passing_action'

    rospy.wait_for_service(server, timeout=3.0)
    try:
      client = rospy.ServiceProxy(server, Trigger)
      req = TriggerRequest()
      resp1  = client(req)
      if resp1.success:
        print("Passing Successed")
      else:
        print("Passing Failed")

      ## Pass
      self.RobotShoot(88, 0)

    except rospy.ServiceException, e:
      print("Service call failed: {}".format(e))

  def MulticastReceiver(self, r2_data, r3_data):
    Robot.sync_last_time = time.time()
    self.robot2['ball_is_handled'] = r2_data.ball_is_handled
    self.robot2['ball_dis']        = r2_data.ball_dis
    self.robot2['position']['x']   = r2_data.position.linear.x
    self.robot2['position']['y']   = r2_data.position.linear.y
    self.robot2['position']['yaw'] = r2_data.position.angular.z
    self.robot2['state']           = r2_data.state
    self.robot3['ball_is_handled'] = r3_data.ball_is_handled
    self.robot3['ball_dis']        = r3_data.ball_dis
    self.robot3['position']['x']   = r3_data.position.linear.x
    self.robot3['position']['y']   = r3_data.position.linear.y
    self.robot3['position']['yaw'] = r3_data.position.angular.z
    self.robot3['state']           = r3_data.state

    if "robot1" in rospy.get_namespace():
      dd12 = np.linalg.norm(np.array([self.__robot_info['location']['x'] - self.robot2['position']['x'],
                                      self.__robot_info['location']['y'] - self.robot2['position']['y']]))
      dd13 = np.linalg.norm(np.array([self.__robot_info['location']['x'] - self.robot3['position']['x'],
                                      self.__robot_info['location']['y'] - self.robot3['position']['y']]))
      self.near_robot_ns = "/robot2" if dd12 < dd13 else "/robot3"
      self.near_robot = self.robot2 if dd12 < dd13 else self.robot3
    elif "robot2" in rospy.get_namespace():
      self.near_robot_ns = "/robot3"
      self.near_robot = self.robot3
    elif "robot3" in rospy.get_namespace():
      self.near_robot_ns = "/robot2"
      self.near_robot = self.robot2

  def MulticastReceiverPass(self, r1_data, r2_data, r3_data):
    self.R1_PassRequestPass            = r1_data.data
    self.R3_PassRequestPass            = r3_data.data
    self.R2_PassRequestPass            = r2_data.data
    
    if not "robot1" in rospy.get_namespace():
      if self.R1_PassRequestPass :
        self.Other_PassRequestPass =True
    elif not "robot2" in rospy.get_namespace():
      if self.R2_PassRequestPass :
        self.Other_PassRequestPass =True
    elif not "robot3" in rospy.get_namespace():
      if self.R3_PassRequestPass :
        self.Other_PassRequestPass =True
    else:
      self.Other_PassRequestPass =False
  
  def MulticastReceiverCatch(self, r2_data, r3_data):
    self.R3_PassRequestCatch            = r3_data.data
    self.R2_PassRequestCatch            = r2_data.data
    if not "robot3" in rospy.get_namespace():
      if self.R3_PassRequestCatch ==True:
        self.Other_PassRequestCatch =True
    elif not "robot2" in rospy.get_namespace():  
      if self.R2_PassRequestCatch ==True:
        self.Other_PassRequestCatch =True
    else:
      self.Other_PassRequestCatch =False
 

  def Supervisor(self):
    duration = time.time() - Robot.sync_last_time
    if duration > 5:
      #print("Lossing Connection with teammates...{}".format(duration), end='\r')
      self.SetMyRole(rospy.get_param('core/role'))
    else:
      if self.MyRole() is "Catcher" or self.MyRole is "Passer":
        if self.robot2['ball_is_handled']:
          self.r2_role = "Attacker"
          self.r3_role = "Supporter"
        elif self.robot3['ball_is_handled']:
          self.r2_role = "Supporter"
          self.r3_role = "Attacker"
        elif time.time() - Robot.passing_last_time > 5:
          ## Passing Timeout
          self.r2_role = "Attacker" if self.robot2['ball_dis'] < self.robot3['ball_dis'] else "Supporter"
          self.r3_role = "Supporter" if self.r2_role is "Attacker" else "Attacker"
      else:
        if self.robot2['ball_is_handled']:
          self.r2_role = "Attacker"
          self.r3_role = "Supporter"
        elif self.robot3['ball_is_handled']:
          self.r2_role = "Supporter"
          self.r3_role = "Attacker"
        else:
          self.r2_role = "Attacker" if self.robot2['ball_dis'] < self.robot3['ball_dis'] else "Supporter"
          self.r3_role = "Supporter" if self.r2_role is "Attacker" else "Attacker"


  def GetState(self, robot_ns):
    if "robot1" in robot_ns.lower():
      return self.robot1
    elif "robot2" in robot_ns.lower():
      return self.robot2
    elif "robot3" in robot_ns.lower():
      return self.robot3
    elif "near_robot" in robot_ns.lower():
      return self.near_robot
    else:
      print("Wrong Namespace")

  def MyState(self):
    if "robot1" in rospy.get_namespace():
      return self.robot1
    elif "robot2" in rospy.get_namespace():
      return self.robot2
    elif "robot3" in rospy.get_namespace():
      return self.robot3
    else:
      print("Wrong Namespace")

  def MyRole(self):
    if "robot1" in rospy.get_namespace():
      return self.r1_role
    elif "robot2" in rospy.get_namespace():
      return self.r2_role
    elif "robot3" in rospy.get_namespace():
      return self.r3_role
    else:
      print("Wrong Namespace")
      return "Wrong Namespace"

  def SetMyRole(self, role):
    if "robot1" in rospy.get_namespace():
      self.r1_role = role
    elif "robot2" in rospy.get_namespace():
      self.r2_role = role
    elif "robot3" in rospy.get_namespace():
      self.r3_role = role
    else:
      print("Wrong Namespace")


  def _Publisher(self, topic, mtype):
    return rospy.Publisher(topic, mtype, queue_size=1)

  def _Getre(self, topic):
    self.__requestsignalin = topic.data
  

  def _GetVision(self, vision):
    rbx = vision.ball_dis * math.cos(math.radians(vision.ball_ang))
    rby = vision.ball_dis * math.sin(math.radians(vision.ball_ang))
    rrbx, rrby = self.Rotate(rbx, rby, self.__robot_info['location']['yaw'])
    gbx = rrbx + self.__robot_info['location']['x']
    gby = rrby + self.__robot_info['location']['y']
    if time.time() - Robot.ball_last_time >= 0.5:
      spx = (gbx - self.__object_info['ball']['global_x']) * 0.02 # m / 0.5s
      spy = (gby - self.__object_info['ball']['global_y']) * 0.02
      spwmx, spwmy = self.ConvertSpeedToPWM(spx, spy)
      self.__object_info['ball']['speed_x']  = spx
      self.__object_info['ball']['speed_y']  = spy
      self.__object_info['ball']['speed_pwm_x']  = spwmx
      self.__object_info['ball']['speed_pwm_y']  = spwmy
      self.__object_info['ball']['global_x'] = gbx
      self.__object_info['ball']['global_y'] = gby
      # print(spwmx, spwmy)
      Robot.ball_last_time = time.time()

    self.__object_info['ball']['dis']    = vision.ball_dis
    self.__object_info['ball']['ang']    = vision.ball_ang
    self.__object_info['Blue']['dis']    = vision.blue_fix_dis
    self.__object_info['Blue']['ang']    = vision.blue_fix_ang
    self.__object_info['Yellow']['dis']  = vision.yellow_fix_dis
    self.__object_info['Yellow']['ang']  = vision.yellow_fix_ang

    if self.__object_info['ball']['dis'] <= self.__handle_dis and abs(self.__object_info['ball']['ang']) <= self.__handle_ang:
      self.__ball_is_handled = True
    else:
      self.__ball_is_handled = False

  def _GetTwopoint(self,vision):
    self.__twopoint_info['Blue']['right']   = vision.blue_right
    self.__twopoint_info['Blue']['left']    = vision.blue_left
    self.__twopoint_info['Yellow']['right'] = vision.yellow_right
    self.__twopoint_info['Yellow']['left']  = vision.yellow_left
    self.RE =vision.requestsignal
    print(RE+'!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1')
  def _GetBlackItemInfo(self, vision):
    self.__obstacle_info['ranges'] =vision.data

  def _GetImu(self, imu_3d):
    front_ang = math.degrees(imu_3d.yaw) + 90 
    self.__robot_info['imu_3d']['yaw'] = imu_3d.yaw  #caculate front angle by imu

  def _GetPosition(self,loc):
    self.__robot_info['location']['x'] = loc.pose.pose.position.x*100
    self.__robot_info['location']['y'] = loc.pose.pose.position.y*100
    qx = loc.pose.pose.orientation.x
    qy = loc.pose.pose.orientation.y
    qz = loc.pose.pose.orientation.z
    qw = loc.pose.pose.orientation.w
    self.__robot_info['location']['yaw'] = math.atan2(2 * (qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)\
                                           / math.pi * 180  #caculate front angle by self_localization
  def Requestsignal(self):
    self.requestsignal = self.OtherRobotdis()

  def RobotStatePub(self, state, PASSREQUESTPass,PASSREQUESTCatch):
    m = RobotState()
    m.state = state
    m.ball_is_handled = self.__ball_is_handled
    m.ball_dis = self.__object_info['ball']['dis']
    m.position.linear.x  = self.__robot_info['location']['x']
    m.position.linear.y  = self.__robot_info['location']['y']
    m.position.angular.z = self.__robot_info['location']['yaw']
    self.state_pub.publish(m)
    self.PassRequestPass_pub.publish(PASSREQUESTPass)
    self.PassRequestCatch_pub.publish(PASSREQUESTCatch)

    i=10
    if self.requestsignal:
      i=0
      self.requestsignal_pub.publish(True)
    elif i<6:
      self.requestsignal_pub.publish(True)
    else :
      self.requestsignal_pub.publish(False)
    
  def ConvertSpeedToPWM(self, x, y):
    reducer = 24
    max_rpm = 7580
    wheel_radius  = 0.11
    circumference = 2 * math.pi * wheel_radius
    _x = (x / circumference * reducer * 60)/max_rpm * 100
    _y = (y / circumference * reducer * 60)/max_rpm * 100
    return _x, _y

  def Rotate(self, x, y, theta):
    _x = x*math.cos(math.radians(theta)) - y*math.sin(math.radians(theta))
    _y = x*math.sin(math.radians(theta)) + y*math.cos(math.radians(theta))
    return _x, _y

  def ConvertSpeedToPWM(self, x, y):
    reducer = 24
    max_rpm = 7580
    wheel_radius  = 0.11
    circumference = 2 * math.pi * wheel_radius
    _x = (x / circumference * reducer * 60)/max_rpm * 100
    _y = (y / circumference * reducer * 60)/max_rpm * 100
    return _x, _y

  def RobotCtrlS(self, x, y, yaw, pass_through=False):
    if pass_through:
      msg = Twist()
      output_x, output_y = self.Rotate(x, y, ROTATE_V_ANG)
      msg.linear.x   = output_x
      msg.linear.y   = output_y
      msg.angular.z  = yaw
      print(output_x, output_y, yaw)
      self.cmdvel_pub.publish(msg)
    else:
      current_vector = math.hypot(x, y)
      output_v = self.pid_v(current_vector * -1)
      output_w = self.pid_w(yaw) * -1
      output_w = output_w if abs(output_w) > self.__minimum_w else self.__minimum_w* np.sign(output_w)

      magnitude = math.sqrt(x**2 + y**2)
      if magnitude == 0:
        unit_vector = (0, 0)
      else:
        unit_vector = (x / magnitude, y / magnitude)

      msg = Twist()
      output_x, output_y = self.Rotate(unit_vector[0]*output_v, unit_vector[1]*output_v, ROTATE_V_ANG)

      msg.linear.x   = output_x
      msg.linear.y   = output_y
      msg.angular.z  = output_w
      self.cmdvel_pub.publish(msg)


  def CheckWhoPass(self):
    if self.R1_PassRequestPass :
      robotpass_x   = self.GetRobot1()['position']['x']
      robotpass_y   = self.GetRobot1()['position']['y']
      robotpass_yaw = self.GetRobot1()['position']['yaw']
    elif self.R2_PassRequestPass :
      robotpass_x   = self.GetRobot2()['position']['x']
      robotpass_y   = self.GetRobot2()['position']['y']
      robotpass_yaw = self.GetRobot2()['position']['yaw']      
    elif self.R3_PassRequestPass :
      robotpass_x   = self.GetRobot3()['position']['x']
      robotpass_y   = self.GetRobot3()['position']['y']
      robotpass_yaw = self.GetRobot3()['position']['yaw']
    else :
      return 0,0,0
    return robotpass_x,robotpass_y,robotpass_yaw
  def CheckWhoCatch(self):
    if   self.R2_PassRequestCatch :
      robotcatch_x   = self.GetRobot2()['position']['x']
      robotcatch_y   = self.GetRobot2()['position']['y']
      robotcatch_yaw = self.GetRobot2()['position']['yaw']
    elif self.R3_PassRequestCatch :
      robotcatch_x   = self.GetRobot3()['position']['x']
      robotcatch_y   = self.GetRobot3()['position']['y']
      robotcatch_yaw = self.GetRobot3()['position']['yaw'] 
    else :
      return 0,0,0       

    return robotcatch_x,robotcatch_y,robotcatch_yaw
  def OtherRobotdis(self):
    robot1_x,robot1_y,robot1_yaw = self.CheckWhoPass()
    robot2_x,robot2_y,robot2_yaw = self.CheckWhoCatch()
    
    dis_x = robot2_x - robot1_x
    dis_y = robot2_y - robot1_y
    dis = pow(pow(dis_x,2) + pow(dis_y,2),0.5)
    if(dis!=0):
      a = math.acos(dis_x/dis)*180/math.pi
      b = math.asin(dis_y/dis)*180/math.pi
    else :
      return False

    if a>=0 and b>=0:
      v_yawa = a
    elif a<=0 and b>=0:
      v_yawa = a+90
    elif a<=0 and b<=0:
      v_yawa = -abs(a+90)
    elif a>=0 and b<=0:
      v_yawa = -a

    v_yawaa = v_yawa-robot1_yaw
    if v_yawaa< -180:
      v_yawaa=v_yawaa+360
    elif v_yawaa>180:
      v_yawaa=v_yawaa-360


    dis_x = robot1_x - robot2_x
    dis_y = robot1_y - robot2_y
    dis = pow(pow(dis_x,2) + pow(dis_y,2),0.5)
    if(dis!=0):
      a = math.acos(dis_x/dis)*180/math.pi
      b = math.asin(dis_y/dis)*180/math.pi
    else :
      return False

    if a>=0 and b>=0:
      v_yawb = a
    elif a<=0 and b>=0:
      v_yawb = a+90
    elif a<=0 and b<=0:
      v_yawb = -abs(a+90)
    elif a>=0 and b<=0:
      v_yawb = -a

    v_yawbb = v_yawb-robot2_yaw
    if v_yawbb< -180:
      v_yawbb=v_yawbb+360
    elif v_yawbb>180:
      v_yawbb=v_yawbb-360
     
    #print(v_yawaa,v_yawbb)
    if abs(v_yawaa)<5 and  abs(v_yawbb)<5:
      return True
    else:
      return False
      
  def GetObjectInfo(self):
    return self.__object_info

  def GetRobotInfo(self):
    return self.__robot_info
 
  def GetTwopoint(self):
    return self.__twopoint_info

  def GetObstacleInfo(self):
    return self.__obstacle_info

  def GetRobotOther(self):
    return self.near_robot
  def GetRobot1(self):
    return self.robot1    
  def GetRobot2(self):
    return self.robot2
  def GetRobot3(self):
    return self.robot3   
  def RealShoot(self, power, pos) :
    msg = Int32()
    msg.data = power
    self.shoot_pub.publish(msg)

  def SimBallHandle(self):
    self.sim_hold_pub.publish(True)
    return self.__ball_is_handled

  def _CheckBallHandle(self, data):
    self.__ball_is_handled = data.data

  def RealBallHandle(self):
    return self.__ball_is_handled
  def GetREInfo(self):
    return self.__requestsignalin


