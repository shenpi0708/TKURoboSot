#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import numpy as np
from robot.robot import Robot

class Attack(Robot):
  def __init__(self):
    pass

  def ClassicAttacking(self, goal_dis, goal_ang):
    v_x   = goal_dis * math.cos(math.radians(goal_ang))
    v_y   = goal_dis * math.sin(math.radians(goal_ang))
    v_yaw = goal_ang

    return v_x, v_y, v_yaw
    
  def Twopoint(self, goal_dis, goal_ang):
    v_y   = goal_dis * math.cos(math.radians(goal_ang))
    v_x   = goal_dis * math.sin(math.radians(goal_ang))
    v_yaw = goal_ang

    return v_x, v_y, v_yaw