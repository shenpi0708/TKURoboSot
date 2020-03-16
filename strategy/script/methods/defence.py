#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import numpy as np
from robot.robot import Robot



class Defence(Robot):
  

  def __init__(self):
    pass

  def Toball(self,ball_dis, ball_ang, goal_dis, goal_ang):
      ball_x =  ball_dis * math.sin(math.radians(ball_ang))
      ball_y =  ball_dis * math.cos(math.radians(ball_ang))

      door_x =  goal_dis * math.cos(math.radians(goal_ang))
      door_y =  goal_dis * math.sin(math.radians(goal_ang))
      c = goal_dis - (goal_dis - ball_dis)
      if goal_ang <-170 and goal_ang >-180 :
         v_x = v_y = 0
         v_yaw = ball_ang
      else :
         v_x = ball_x
         v_y = ball_y
         v_yaw = ball_ang

      return v_x, v_y , v_yaw