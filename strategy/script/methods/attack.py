#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import numpy as np
from robot.robot import Robot
from robot.obstacle import Obstacle


class Attack(Robot,Obstacle):
  

  def __init__(self):
    pass

  def ClassicAttacking(self, goal_dis, goal_ang):
    v_x   = goal_dis * math.cos(math.radians(goal_ang))
    v_y   = goal_dis * math.sin(math.radians(goal_ang))
    v_yaw = goal_ang

    return v_x, v_y, v_yaw

  def Orbit(self, goal_ang):
    v_x = 0
    v_y = 0
    v_yaw = goal_ang

    return v_x, v_y, v_yaw
    
  def Twopoint(self, goal_dis, goal_ang,outside_ang,):
    v_a   = goal_dis * math.cos(math.radians(goal_ang))
    v_x   = goal_dis * math.sin(math.radians(goal_ang))
    v_yaw = goal_ang
    a=0
    b=0
    if abs(v_yaw - 360) < abs(v_yaw):
      o_yaw = v_yaw - 360
    elif abs(v_yaw + 360) < abs(v_yaw):
      o_yaw = v_yaw + 360
    else:
      o_yaw = v_yaw
    
    if outside_ang <b and b>=0:
      b = 1
    elif outside_ang >b and b<=0:
      b = -1
    else:
      b = 0
    if outside_ang >120 or outside_ang < -120:
      if b > 0 :
        a = -1
      elif b < 0 :
        a = 1
      else :
        a = 0

    if a > 0 :
      v_y = -v_a
    elif a < 0 :
      v_y = v_a
    else :
      v_y = 0  
    return v_x, v_y, o_yaw
  def ball_pass_check(self, goal_dis, goal_ang,ranges, angle_increment):
    
    
    self.__goal_dis = goal_dis
    self.__goal_ang = goal_ang
    self.__ranges = ranges
    self.__angle_increment = angle_increment


    self.raw , object_dis= self.state(ranges) 
    self.edit = self.filter(self.raw)
    if (self.edit)=='':
      return True 
    else:
      return False

  def Post_up(self, goal_dis, goal_ang,ranges, angle_increment):
    
    
    self.__goal_dis = goal_dis
    self.__goal_ang = goal_ang
    self.__ranges = ranges
    self.__angle_increment = angle_increment


    self.raw , object_dis= self.state(ranges) 
    self.edit = self.filter(self.raw)        
    obstacle_force_x , obstacle_force_y = self.Obstacle_segmentation(self.edit ,angle_increment , object_dis)
    
    if obstacle_force_x == 0 and obstacle_force_y == 0 :
        v_x   = goal_dis * math.cos(math.radians(goal_ang))
        v_y   = goal_dis * math.sin(math.radians(goal_ang))
        v_yaw = goal_ang

        return v_x , v_y , v_yaw

    else :
        v_x,v_y,v_yaw = self.Force_Calculation(obstacle_force_x , obstacle_force_y ,goal_ang, goal_dis,1)
  def ball_pass(self, robot1_x,robot1_y,robot1_yaw, robot2_x,robot2_y,robot2_yaw):
    dis_x = robot2_x - robot1_x
    dis_y = robot2_y - robot1_y
    dis = pow(pow(dis_x,2) + pow(dis_y,2),0.5)
    if(dis!=0):
      a = math.acos(dis_x/dis)*180/math.pi
      b = math.asin(dis_y/dis)*180/math.pi
    else :
      print('error')
      return 0, 0, 0
    if a>=0 and b>=0:
      v_yawa = a
    elif a<=0 and b>=0:
      v_yawa = a+90
    elif a<=0 and b<=0:
      v_yawa = -abs(a+90)
    elif a>=0 and b<=0:
      v_yawa = -a
    
    v_yaw = v_yawa-robot1_yaw
    if v_yaw< -180:
      v_yaw=v_yaw+360
    elif v_yaw>180:
      v_yaw=v_yaw-360
    v_yaw*=2  
    return 0,0,v_yaw

    
